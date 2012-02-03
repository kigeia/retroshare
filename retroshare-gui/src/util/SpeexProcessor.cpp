#include "SpeexProcessor.h"

#include <speex/speex.h>
#include <speex/speex_preprocess.h>

using namespace QtSpeex;

SpeexProcessor::SpeexProcessor(QObject *parent) : QIODevice(parent) {
        for (int i=0; i<FRAME_SIZE; i++)
                pcm[i] = 0;
        bits = new SpeexBits;
	speex_bits_init(bits);

        enc_state = speex_encoder_init(&speex_wb_mode);
        int quality = 5;
        speex_encoder_ctl(enc_state, SPEEX_SET_QUALITY, &quality);

        int jitterTime = 0;

        speex_encoder_ctl(enc_state, SPEEX_SET_QUALITY, &quality);

        int on = 1; // on
        speex_encoder_ctl(enc_state, SPEEX_SET_VAD, &on);// VAD = Voice Activity Detection
        speex_encoder_ctl(enc_state, SPEEX_SET_DTX, &on);// DTX = Discontinuous Transmission
        speex_encoder_ctl(enc_state, SPEEX_SET_VBR, &on);// VBR = Variable Bit rate
        float VBRQuality = 4.0; // VBR = Variable Bit rate
        speex_encoder_ctl(enc_state, SPEEX_SET_VBR_QUALITY, &VBRQuality);
        int VBRBitRate = 2 * 1024 * 8; // VBR = Variable Bit rate
        speex_encoder_ctl(enc_state, SPEEX_SET_VBR_MAX_BITRATE, &VBRBitRate);

        dec_state = speex_decoder_init(&speex_wb_mode);
        speex_decoder_ctl(dec_state, SPEEX_SET_ENH, &on);// Set the perceptual enhancement

        preprocessor = speex_preprocess_state_init(FRAME_SIZE, SAMPLING_RATE);

        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_AGC, &on);
        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_VAD, &on);
        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_DENOISE, &on);

        echo_state = speex_echo_state_init(FRAME_SIZE, ECHOTAILSIZE*FRAME_SIZE);
        int tmp = SAMPLING_RATE;
        speex_echo_ctl(echo_state, SPEEX_ECHO_SET_SAMPLING_RATE, &tmp);
        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_ECHO_STATE, echo_state);
}

SpeexProcessor::~SpeexProcessor() {
        speex_preprocess_state_destroy(preprocessor);
        speex_echo_state_destroy(echo_state);

	speex_encoder_destroy(enc_state);
	speex_decoder_destroy(dec_state);

        speex_bits_destroy(bits);
        delete bits;
}

void SpeexProcessor::putNetworkPacket(QByteArray packet) {
	inputNetworkBuffer.append(packet);
}

QByteArray SpeexProcessor::getNetworkPacket() {
	return outputNetworkBuffer.takeFirst();
}

bool SpeexProcessor::hasPendingPackets() {
	return !outputNetworkBuffer.empty();
}

qint64 SpeexProcessor::writeData(const char *data, qint64 maxSize) {

        inputBuffer += QByteArray(data, maxSize);

        while(inputBuffer.size() > FRAME_SIZE * sizeof(qint16)) {
                QByteArray frame = inputBuffer.left(FRAME_SIZE * sizeof(qint16));
                //speex_echo_cancellation(echo_state, (qint16*) frame.data(), psSpeaker, pcm);
                //std::cerr  << "speex_echo_capture called" << std::endl;
                speex_echo_capture(echo_state, (qint16*) frame.data(), pcm);
                bool isSpeech = speex_preprocess_run(preprocessor, pcm);

                speex_bits_reset(bits);
                int needTransmit = speex_encode_int(enc_state, pcm, bits);
                if (isSpeech && needTransmit) {
                    QByteArray networkFrame;
                    networkFrame.resize(speex_bits_nbytes(bits));
                    speex_bits_write(bits, networkFrame.data(), networkFrame.size());

                    outputNetworkBuffer.append(networkFrame);
                    emit networkPacketReady();
                }

                inputBuffer = inputBuffer.right(inputBuffer.size() - FRAME_SIZE * sizeof(qint16));
	}

	return maxSize;
}

qint64 SpeexProcessor::readData(char *data, qint64 maxSize) {
	if(outputBuffer.size() > 0 || !inputNetworkBuffer.empty()) {
		while(outputBuffer.size() < maxSize && !inputNetworkBuffer.empty()) {
			QByteArray packet = inputNetworkBuffer.takeFirst();

			QByteArray frame;
                        frame.resize(FRAME_SIZE * sizeof(qint16));

			speex_bits_read_from(bits, packet.data(), packet.size());
                        speex_decode_int(dec_state, bits, (qint16*)  frame.data());
			outputBuffer += frame;
                        //std::cerr << "speex_echo_playback called" << std::endl;
                        speex_echo_playback(echo_state, (qint16*)  frame.data());
                }
	} else {
		// perform extrapolation
		while(outputBuffer.size() < maxSize) {
			QByteArray frame;
                        frame.resize(FRAME_SIZE * sizeof(qint16));

			speex_decode_int(dec_state, NULL, (qint16*) frame.data());

                        outputBuffer += frame;
                        //std::cerr << "speex_echo_playback called" << std::endl;
                        speex_echo_playback(echo_state, (qint16*)  frame.data());
                }
	}

	QByteArray resultBuffer = outputBuffer.left(maxSize);
	memcpy(data, resultBuffer.data(), resultBuffer.size());

	outputBuffer = outputBuffer.right(outputBuffer.size() - resultBuffer.size());

        return resultBuffer.size();
}

bool SpeexProcessor::isSequential() const {
	return true;
}

void SpeexProcessor::speex_jitter_init(void *decoder, int sampling_rate)
{
   jitter->dec = decoder;
   speex_decoder_ctl(decoder, SPEEX_GET_FRAME_SIZE, &jitter->frame_size);

   jitter->packets = jitter_buffer_init(jitter->frame_size);

   speex_bits_init(jitter->current_packet);
   jitter->valid_bits = 0;
}

void SpeexProcessor::speex_jitter_destroy()
{
   jitter_buffer_destroy(jitter->packets);
   speex_bits_destroy(jitter->current_packet);
}

void SpeexProcessor::speex_jitter_put(char *packet, int len, int timestamp)
{
   JitterBufferPacket p;
   p.data = packet;
   p.len = len;
   p.timestamp = timestamp;
   p.span = jitter->frame_size;
   jitter_buffer_put(jitter->packets, &p);
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

   if (jitter->valid_bits)
   {
      /* Try decoding last received packet */
      ret = speex_decode_int(jitter->dec, jitter->current_packet, out);
      if (ret == 0)
      {
         jitter_buffer_tick(jitter->packets);
         return;
      } else {
         jitter->valid_bits = 0;
      }
   }

   if (current_timestamp)
    ret = jitter_buffer_get(jitter->packets, &packet, jitter->frame_size, current_timestamp);
   else
    ret = jitter_buffer_get(jitter->packets, &packet, jitter->frame_size, NULL);

   if (ret != JITTER_BUFFER_OK)
   {
      /* No packet found */
      speex_decode_int(jitter->dec, NULL, out);
   } else {
      speex_bits_read_from(jitter->current_packet, packet.data, packet.len);
      /* Decode packet */
      ret = speex_decode_int(jitter->dec, jitter->current_packet, out);
      if (ret == 0)
      {
         jitter->valid_bits = 1;
      } else {
         /* Error while decoding */
         for (i=0;i<jitter->frame_size;i++)
            out[i]=0;
      }
   }
   speex_decoder_ctl(jitter->dec, SPEEX_GET_ACTIVITY, &activity);
   if (activity < 30)
   {
      jitter_buffer_update_delay(jitter->packets, &packet, NULL);
   }
   jitter_buffer_tick(jitter->packets);
   //ret = jitter_buffer_ctl(jitter->packets, JITTER_BUFFER_GET_AVALIABLE_COUNT, &bufferCount);
   //sprintf(msg, “   get %d bufferCount=%d\n”, speex_jitter_get_pointer_timestamp(jitter), bufferCount);
   //debugPrint(msg);
}

int SpeexProcessor::speex_jitter_get_pointer_timestamp()
{
   return jitter_buffer_get_pointer_timestamp(jitter->packets);
}
