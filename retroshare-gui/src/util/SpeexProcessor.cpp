#include "SpeexProcessor.h"

#include <speex/speex.h>
#include <speex/speex_preprocess.h>

#include <sstream>
#include <iostream>
#include <fstream>
#include <fcntl.h>

using namespace QtSpeex;

SpeexProcessor::SpeexProcessor(QObject *parent) : QIODevice(parent),
    preprocessor(0),
    enc_state(0),
    enc_bits(),
    send_timestamp(0),
    echo_state(0),
    dec_state(0),
    dec_bits(),

    jitter(),
    mostUpdatedTSatPut(0),
    firsttimecalling_get(true),
    inputBuffer(),
    outputBuffer()
{
        for (int i=0; i<FRAME_SIZE; i++)
                pcm[i] = 0;
        enc_bits = new SpeexBits;
        speex_bits_init(enc_bits);
        dec_bits = new SpeexBits;
        speex_bits_init(dec_bits);

        enc_state = speex_encoder_init(&speex_wb_mode);
        int tmp = 8;
        speex_encoder_ctl(enc_state, SPEEX_SET_QUALITY, &tmp);
        tmp = 10;
        speex_encoder_ctl(enc_state, SPEEX_SET_COMPLEXITY, &tmp);

        int on = 1; // on
        speex_encoder_ctl(enc_state, SPEEX_SET_VAD, &on);// VAD = Voice Activity Detection
        speex_encoder_ctl(enc_state, SPEEX_SET_DTX, &on);// DTX = Discontinuous Transmission
        speex_encoder_ctl(enc_state, SPEEX_SET_VBR, &on);// VBR = Variable Bit rate
        float VBRQuality = 8.0; // VBR = Variable Bit rate
        speex_encoder_ctl(enc_state, SPEEX_SET_VBR_QUALITY, &VBRQuality);
        int VBRBitRate = 1024 * 8 * 5 / 2; // VBR = Variable Bit rate 2,5 kb/s
        speex_encoder_ctl(enc_state, SPEEX_SET_VBR_MAX_BITRATE, &VBRBitRate);

        dec_state = speex_decoder_init(&speex_wb_mode);
        speex_decoder_ctl(dec_state, SPEEX_SET_ENH, &on);// Set the perceptual enhancement
        speex_jitter_init(&jitter, dec_state, SAMPLING_RATE);


        preprocessor = speex_preprocess_state_init(FRAME_SIZE, SAMPLING_RATE);
        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_AGC, &on);
        tmp = 2;
        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_AGC_MAX_GAIN, &tmp);
        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_VAD, &on);
        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_DENOISE, &on);
        tmp = -30;
        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_NOISE_SUPPRESS, &tmp);

        echo_state = speex_echo_state_init(FRAME_SIZE, ECHOTAILSIZE*FRAME_SIZE);
        tmp = SAMPLING_RATE;
        speex_echo_ctl(echo_state, SPEEX_ECHO_SET_SAMPLING_RATE, &tmp);
        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_ECHO_STATE, echo_state);
}

SpeexProcessor::~SpeexProcessor() {
        speex_preprocess_state_destroy(preprocessor);
        speex_echo_state_destroy(echo_state);

	speex_encoder_destroy(enc_state);
	speex_decoder_destroy(dec_state);

        speex_jitter_destroy();

        speex_bits_destroy(enc_bits);
        speex_bits_destroy(dec_bits);
        delete enc_bits;
        delete dec_bits;
}

void SpeexProcessor::putNetworkPacket(QByteArray packet) {
    //buffer:
    //  encodedBufSize + 4 | timestamp | encodedBuf
    // ——————–———–——————–———–——————–———–——————–———–——————–
    //  4                  |  4        | encodedBufSize – 4
    //the size part (first 4 byets) is not actually used in the logic
    if (packet.size() > 8)
    {
        int recv_timestamp = ((int*)packet.data())[1];
        mostUpdatedTSatPut = recv_timestamp;
        if (firsttimecalling_get)
            return;
        speex_jitter_put((char *)packet.data()+8, packet.size()-8, recv_timestamp);
    }
}

QByteArray SpeexProcessor::getNetworkPacket() {
	return outputNetworkBuffer.takeFirst();
}

bool SpeexProcessor::hasPendingPackets() {
	return !outputNetworkBuffer.empty();
}

//make it quiet because it spams too much the standard ouptut and error
void quiet_speex_echo_capture (SpeexEchoState *st, const spx_int16_t *rec, spx_int16_t *out) {
    int orig_fd = dup(2);
    int fd = open("/dev/null", O_WRONLY);
    dup2(fd, 2);
    speex_echo_capture(st, rec, out);
    close(fd);
    dup2(orig_fd, 2);
    close(orig_fd);
}

qint64 SpeexProcessor::writeData(const char *data, qint64 maxSize) {
        inputBuffer += QByteArray(data, maxSize);

        while(inputBuffer.size() > FRAME_SIZE * sizeof(qint16)) {
                QByteArray frame = inputBuffer.left(FRAME_SIZE * sizeof(qint16)+8);//add 4 for the frame timestamp for the jitter buffer and the packet size

                quiet_speex_echo_capture(echo_state, (qint16*) frame.data(), pcm);
                bool isSpeech = speex_preprocess_run(preprocessor, pcm);

                speex_bits_reset(enc_bits);
                int needTransmit = speex_encode_int(enc_state, pcm, enc_bits);
                if (isSpeech && needTransmit) {
                    QByteArray networkFrame;
                    networkFrame.resize(speex_bits_nbytes(enc_bits));
                    int packetSize = speex_bits_write(enc_bits, networkFrame.data()+8, networkFrame.size()-8);

                    ((int*)networkFrame.data())[0] = packetSize+4;
                    ((int*)networkFrame.data())[1] = send_timestamp;

                    outputNetworkBuffer.append(networkFrame);
                    emit networkPacketReady();
                }
                send_timestamp += FRAME_SIZE;
                if (send_timestamp >= INT_MAX)
                    send_timestamp = 0;

                inputBuffer = inputBuffer.right(inputBuffer.size() - FRAME_SIZE * sizeof(qint16));
	}

	return maxSize;
}

//make it quiet because it spams too much the standard error
void quiet_speex_echo_playback (SpeexEchoState *st, const spx_int16_t *play) {
    int orig_fd = dup(2);
    int fd = open("/dev/null", O_WRONLY);
    dup2(fd, 2);
    speex_echo_playback(st, play);
    close(fd);
    dup2(orig_fd, 2);
    close(orig_fd);
}

qint64 SpeexProcessor::readData(char *data, qint64 maxSize) {
    int ts = 0; //time stamp for the jitter call
    if (firsttimecalling_get)
    {
        int ts = mostUpdatedTSatPut;
        firsttimecalling_get = false;
    }

    while(outputBuffer.size() < maxSize) {
        QByteArray frame;
        frame.resize(FRAME_SIZE * sizeof(qint16));
        speex_jitter_get((spx_int16_t*)frame.data(), &ts);
        outputBuffer += frame;
        quiet_speex_echo_playback(echo_state, (qint16*)  frame.data());
    }

    QByteArray resultBuffer = outputBuffer.left(maxSize);
    memcpy(data, resultBuffer.data(), resultBuffer.size());

    outputBuffer = outputBuffer.right(outputBuffer.size() - resultBuffer.size());

    return resultBuffer.size();
}

bool SpeexProcessor::isSequential() const {
	return true;
}

void SpeexProcessor::speex_jitter_init(SpeexJitter *jit, void *decoder, int sampling_rate)
{
   jit->dec = decoder;
   speex_decoder_ctl(decoder, SPEEX_GET_FRAME_SIZE, &jit->frame_size);

   jit->packets = jitter_buffer_init(jit->frame_size);
    jit->current_packet = new SpeexBits;
   speex_bits_init(jit->current_packet);
   jit->valid_bits = 0;
}

void SpeexProcessor::speex_jitter_destroy()
{
   jitter_buffer_destroy(jitter.packets);
   speex_bits_destroy(jitter.current_packet);
}

void SpeexProcessor::speex_jitter_put(char *packet, int len, int timestamp)
{
   JitterBufferPacket p;
   p.data = packet;
   p.len = len;
   p.timestamp = timestamp;
   p.span = jitter.frame_size;
   jitter_buffer_put(jitter.packets, &p);
}

void SpeexProcessor::speex_jitter_get(spx_int16_t *out, int *current_timestamp)
{
   int i;
   int ret;
   spx_int32_t activity;
   int bufferCount = 0;
   JitterBufferPacket packet;
   //char data[40960];
   //packet.data = data;
   //packet.len = 40960;
   packet.data = reinterpret_cast<char *>(psSpeaker);
   packet.len = 320 * ECHOTAILSIZE * 10;

   if (jitter.valid_bits)
   {
      /* Try decoding last received packet */
      ret = speex_decode_int(jitter.dec, jitter.current_packet, out);
      if (ret == 0)
      {
         jitter_buffer_tick(jitter.packets);
         return;
      } else {
         jitter.valid_bits = 0;
      }
   }

   if (current_timestamp)
    ret = jitter_buffer_get(jitter.packets, &packet, jitter.frame_size, current_timestamp);
   else
    ret = jitter_buffer_get(jitter.packets, &packet, jitter.frame_size, NULL);

   if (ret != JITTER_BUFFER_OK)
   {
      /* No packet found */
      speex_decode_int(jitter.dec, NULL, out);
   } else {
      speex_bits_read_from(jitter.current_packet, packet.data, packet.len);
      /* Decode packet */
      ret = speex_decode_int(jitter.dec, jitter.current_packet, out);
      if (ret == 0)
      {
         jitter.valid_bits = 1;
      } else {
         /* Error while decoding */
         for (i=0;i<jitter.frame_size;i++)
            out[i]=0;
      }
   }
   speex_decoder_ctl(jitter.dec, SPEEX_GET_ACTIVITY, &activity);
   if (activity < 30)
   {
      jitter_buffer_update_delay(jitter.packets, &packet, NULL);
   }
   jitter_buffer_tick(jitter.packets);
   //ret = jitter_buffer_ctl(jitter.packets, JITTER_BUFFER_GET_AVALIABLE_COUNT, &bufferCount);
   //sprintf(msg, “   get %d bufferCount=%d\n”, speex_jitter_get_pointer_timestamp(jitter), bufferCount);
   //debugPrint(msg);
}

int SpeexProcessor::speex_jitter_get_pointer_timestamp()
{
   return jitter_buffer_get_pointer_timestamp(jitter.packets);
}
