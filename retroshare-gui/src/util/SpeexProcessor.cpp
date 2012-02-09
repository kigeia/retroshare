#include "SpeexProcessor.h"

#include <speex/speex.h>
#include <speex/speex_preprocess.h>

#include <sstream>
#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <math.h>
#include <cstdlib>

#include "gui/settings/rsharesettings.h"
#include "audiodevicehelper.h"

#define iroundf(x) ( static_cast<int>(x) )

using namespace QtSpeex;

static QAudioInput *qAudioInput = NULL;

SpeexInputProcessor::SpeexInputProcessor(QObject *parent) : QIODevice(parent),
    preprocessor(0),
    enc_state(0),
    enc_bits(),
    send_timestamp(0),
    echo_state(0),
    inputBuffer(),
    iMaxBitRate(16800),
    bResetProcessor(true)
{
        for (int i=0; i<FRAME_SIZE; i++)
                pcm[i] = 0;

        enc_bits = new SpeexBits;
        speex_bits_init(enc_bits);
        speex_bits_reset(enc_bits);
        enc_state = speex_encoder_init(&speex_wb_mode);

        int iArg=1;
        speex_encoder_ctl(enc_state,SPEEX_SET_VBR, &iArg);


        iArg = 0;
        speex_encoder_ctl(enc_state,SPEEX_SET_VAD, &iArg);
        speex_encoder_ctl(enc_state,SPEEX_SET_DTX, &iArg);

        float fArg=8.0;
        speex_encoder_ctl(enc_state,SPEEX_SET_VBR_QUALITY, &fArg);

        iArg = iMaxBitRate;
        speex_encoder_ctl(enc_state, SPEEX_SET_VBR_MAX_BITRATE, &iArg);

        iArg = 10;
        speex_encoder_ctl(enc_state,SPEEX_SET_COMPLEXITY, &iArg);

        iArg = 10;
        speex_encoder_ctl(enc_state,SPEEX_SET_QUALITY, &iArg);

        /*echo_state = speex_echo_state_init(FRAME_SIZE, ECHOTAILSIZE*FRAME_SIZE);
        iArg = SAMPLING_RATE;
        speex_echo_ctl(echo_state, SPEEX_ECHO_SET_SAMPLING_RATE, &iArg);
        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_ECHO_STATE, echo_state);*/


        //iEchoFreq = iMicFreq = iSampleRate;

        iFrameCounter = 0;
        iSilentFrames = 0;
        iHoldFrames = 0;

        bResetProcessor = true;

        //bEchoMulti = false;

        preprocessor = NULL;
        echo_state = NULL;
        //srsMic = srsEcho = NULL;
        //iJitterSeq = 0;
        //iMinBuffered = 1000;

        //psMic = new short[iFrameSize];
        //psClean = new short[iFrameSize];

        //psSpeaker = NULL;

        //iEchoChannels = iMicChannels = 0;
        //iEchoFilled = iMicFilled = 0;
        //eMicFormat = eEchoFormat = SampleFloat;
        //iMicSampleSize = iEchoSampleSize = 0;

        bPreviousVoice = false;

        //pfMicInput = pfEchoInput = pfOutput = NULL;

        iRealTimeBitrate = 0;
        dPeakSignal = dPeakSpeaker = dPeakMic = dPeakCleanMic = dVoiceAcivityLevel = 0.0;

        //if (g.uiSession) {
        //TODO : get the maxbitrate from a rs service or a dynamic code
        //iMaxBitRate = 10000;
        //}

        //bRunning = true;
        if (!qAudioInput) {
            qAudioInput = AudioDeviceHelper::getPreferedInputDevice();
        }
        if (qAudioInput) {
            qAudioInput->start(this);
        } else {
            std::cerr << "Unable to find input device." << std::endl;
        }
        this->open(QIODevice::WriteOnly | QIODevice::Unbuffered);
    }

SpeexInputProcessor::~SpeexInputProcessor() {
        if (qAudioInput) {
            qAudioInput->stop();
        }
        close();
        speex_preprocess_state_destroy(preprocessor);
        speex_echo_state_destroy(echo_state);

        speex_encoder_destroy(enc_state);


        speex_bits_destroy(enc_bits);
        delete enc_bits;
}

QByteArray SpeexInputProcessor::getNetworkPacket() {
        return outputNetworkBuffer.takeFirst();
}

bool SpeexInputProcessor::hasPendingPackets() {
        return !outputNetworkBuffer.empty();
}

void SpeexInputProcessor::stopAudioInputDevice() {
        if (qAudioInput) {
            qAudioInput->stop();
        }
}

void SpeexInputProcessor::startAudioInputDevice() {
        if (qAudioInput) {
            qAudioInput->start(this);
        }
}

QAudioInput* getAudioInputDevice() {
    return qAudioInput;
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

qint64 SpeexInputProcessor::writeData(const char *data, qint64 maxSize) {
        int iArg;
        int i;
        float sum;
        short max;

        inputBuffer += QByteArray(data, maxSize);

        while(inputBuffer.size() > FRAME_SIZE * sizeof(qint16)) {

                QByteArray source_frame = inputBuffer.left(FRAME_SIZE * sizeof(qint16));
                const short* psMic = (const short *)source_frame.data();

                //quiet_speex_echo_capture(echo_state, psMic, pcm);
                //let's do volume detection
                iFrameCounter++;
                sum=1.0f;
                for (i=0;i<FRAME_SIZE;i++) {
                        sum += static_cast<float>(psMic[i] * psMic[i]);
                }
                dPeakMic = qMax(20.0f*log10f(sqrtf(sum / static_cast<float>(FRAME_SIZE)) / 32768.0f), -96.0f);

                max = 1;
                for (i=0;i<FRAME_SIZE;i++)
                        max = static_cast<short>(std::abs(psMic[i]) > max ? std::abs(psMic[i]) : max);
                dMaxMic = max;

                dPeakSpeaker = 0.0;

                QMutexLocker l(&qmSpeex);

                if (bResetProcessor) {
                        if (preprocessor)
                                speex_preprocess_state_destroy(preprocessor);
                        if (echo_state)
                                speex_echo_state_destroy(echo_state);

                        preprocessor = speex_preprocess_state_init(FRAME_SIZE, SAMPLING_RATE);

                        iArg = 1;
                        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_VAD, &iArg);
                        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_AGC, &iArg);
                        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_DENOISE, &iArg);
                        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_DEREVERB, &iArg);

                        iArg = 30000;
                        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_AGC_TARGET, &iArg);

                        iArg = -60;
                        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_AGC_DECREMENT, &iArg);

                        iArg = Settings->getVoipiNoiseSuppress();
                        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_NOISE_SUPPRESS, &iArg);

                        /*if (iEchoChannels > 0) {
                                sesEcho = speex_echo_state_init_mc(iFrameSize, iFrameSize*10, 1, bEchoMulti ? iEchoChannels : 1);
                                iArg = iSampleRate;
                                speex_echo_ctl(sesEcho, SPEEX_ECHO_SET_SAMPLING_RATE, &iArg);
                                speex_preprocess_ctl(sppPreprocess, SPEEX_PREPROCESS_SET_ECHO_STATE, sesEcho);

                                qWarning("AudioInput: ECHO CANCELLER ACTIVE");
                        } else {
                                sesEcho = NULL;
                        }*/

                        bResetProcessor = false;
                }

                float v = 30000.0f / static_cast<float>(Settings->getVoipiMinLoudness());

                std::cerr << "Settings->getVoipiMinLoudness() : " << Settings->getVoipiMinLoudness() << std::endl;

                iArg = iroundf(floorf(20.0f * log10f(v)));
                speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_AGC_MAX_GAIN, &iArg);
                std::cerr << "SPEEX_PREPROCESS_SET_AGC_MAX_GAIN : " << iArg << std::endl;

                speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_GET_AGC_GAIN, &iArg);
                float gainValue = static_cast<float>(iArg);
                iArg = Settings->getVoipiNoiseSuppress() - iArg;
                speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_NOISE_SUPPRESS, &iArg);

                /*if (sesEcho && psSpeaker) {
                        speex_echo_cancellation(sesEcho, psMic, psSpeaker, psClean);
                        speex_preprocess_run(sppPreprocess, psClean);
                        psSource = psClean;
                } else {*/
                        speex_preprocess_run(preprocessor, (short int*)source_frame.data());
                        short * psSource = (short int*)source_frame.data();
                //}
                //we will now analize the processed signal
                sum=1.0f;
                for (i=0;i<FRAME_SIZE;i++)
                        sum += static_cast<float>(psSource[i] * psSource[i]);
                float micLevel = sqrtf(sum / static_cast<float>(FRAME_SIZE));
                dPeakSignal = qMax(20.0f*log10f(micLevel / 32768.0f), -96.0f);

                spx_int32_t prob = 0;
                speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_GET_PROB, &prob);//speech probability
                fSpeechProb = static_cast<float>(prob) / 100.0f;

                // clean microphone level: peak of filtered signal attenuated by AGC gain
                dPeakCleanMic = qMax(dPeakSignal - gainValue, -96.0f);
                dVoiceAcivityLevel = 0.4f * fSpeechProb + 0.6f * (1.0f + dPeakCleanMic / 96.0f);//ponderation for speech detection and audio amplitude

                bool bIsSpeech = false;

                if (dVoiceAcivityLevel > (static_cast<float>(Settings->getVoipfVADmax()) / 32767))
                        bIsSpeech = true;
                else if (dVoiceAcivityLevel > (static_cast<float>(Settings->getVoipfVADmin()) / 32767) && bPreviousVoice)
                        bIsSpeech = true;

                if (! bIsSpeech) {
                        iHoldFrames++;
                        if (iHoldFrames < Settings->getVoiceHold())
                                bIsSpeech = true;
                } else {
                        iHoldFrames = 0;
                }


                if (Settings->getVoipATransmit() == RshareSettings::AudioTransmitContinous) {
                        bIsSpeech = true;
                }
                else if (Settings->getVoipATransmit() == RshareSettings::AudioTransmitPushToTalk)
                        bIsSpeech = false;//g.s.uiDoublePush && ((g.uiDoublePush < g.s.uiDoublePush) || (g.tDoublePush.elapsed() < g.s.uiDoublePush));

                //bIsSpeech = bIsSpeech || (g.iPushToTalk > 0);

                /*if (g.s.bMute || ((g.s.lmLoopMode != Settings::Local) && p && (p->bMute || p->bSuppress)) || g.bPushToMute || (g.iTarget < 0)) {
                        bIsSpeech = false;
                }*/

                if (bIsSpeech) {
                        iSilentFrames = 0;
                } else {
                        iSilentFrames++;
                        if (iSilentFrames > 500)
                                iFrameCounter = 0;
                }

                /*if (p) {
                        if (! bIsSpeech)
                                p->setTalking(Settings::Passive);
                        else if (g.iTarget == 0)
                                p->setTalking(Settings::Talking);
                        else
                                p->setTalking(Settings::Shouting);
                }*/


                if (! bIsSpeech && ! bPreviousVoice) {
                        iRealTimeBitrate = 0;
                        /*if (g.s.iIdleTime && ! g.s.bDeaf && ((tIdle.elapsed() / 1000000ULL) > g.s.iIdleTime)) {
                                emit doDeaf();
                                tIdle.restart();
                        }*/
                        spx_int32_t increment = 0;
                        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_AGC_INCREMENT, &increment);
                } else {
                        spx_int32_t increment = 12;
                        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_AGC_INCREMENT, &increment);
                }

                int vbr = 0;
                speex_encoder_ctl(enc_state, SPEEX_GET_VBR_MAX_BITRATE, &vbr);
                if (vbr != iMaxBitRate) {
                        vbr = iMaxBitRate;
                        speex_encoder_ctl(enc_state, SPEEX_SET_VBR_MAX_BITRATE, &vbr);
                }

                if (! bPreviousVoice)
                        speex_encoder_ctl(enc_state, SPEEX_RESET_STATE, NULL);

                speex_bits_reset(enc_bits);
                speex_encode_int(enc_state, psSource, enc_bits);
                if (bIsSpeech) {
                    QByteArray networkFrame;
                    networkFrame.resize(speex_bits_nbytes(enc_bits)+4);//add 4 for the frame timestamp for the jitter buffer
                    int packetSize = speex_bits_write(enc_bits, networkFrame.data()+4, networkFrame.size()-4);
                    ((int*)networkFrame.data())[0] = send_timestamp;

                    outputNetworkBuffer.append(networkFrame);
                    emit networkPacketReady();

                    iRealTimeBitrate = packetSize * SAMPLING_RATE / FRAME_SIZE * 8;
                } else {
                    iRealTimeBitrate = 0;
                }
                bPreviousVoice = bIsSpeech;

                //std::cerr << "iRealTimeBitrate : " << iRealTimeBitrate << std::endl;

                send_timestamp += FRAME_SIZE;
                if (send_timestamp >= INT_MAX)
                    send_timestamp = 0;

                inputBuffer = inputBuffer.right(inputBuffer.size() - FRAME_SIZE * sizeof(qint16));
	}

	return maxSize;
}


SpeexOutputProcessor::SpeexOutputProcessor(QObject *parent) : QIODevice(parent),
    dec_state(0),
    dec_bits(),
    echo_state(0),
    jitter(),
    outputBuffer(),
    mostUpdatedTSatPut(0),
    firsttimecalling_get(true)
{
        dec_bits = new SpeexBits;
        speex_bits_init(dec_bits);
        dec_state = speex_decoder_init(&speex_wb_mode);
        int on = 1;
        speex_decoder_ctl(dec_state, SPEEX_SET_ENH, &on);// Set the perceptual enhancement
        speex_jitter_init(&jitter, dec_state, SAMPLING_RATE);

        /*echo_state = speex_echo_state_init(FRAME_SIZE, ECHOTAILSIZE*FRAME_SIZE);
        int tmp = SAMPLING_RATE;
        speex_echo_ctl(echo_state, SPEEX_ECHO_SET_SAMPLING_RATE, &tmp);*/
}

SpeexOutputProcessor::~SpeexOutputProcessor() {
        //speex_echo_state_destroy(echo_state);

        speex_decoder_destroy(dec_state);


        speex_bits_destroy(dec_bits);
        delete dec_bits;
}

void SpeexOutputProcessor::putNetworkPacket(QByteArray packet) {
    //buffer:
    //  timestamp | encodedBuf
    // —————–———–——————–———–——————–———–——————–
    //    4       | totalSize – 4
    //the size part (first 4 byets) is not actually used in the logic
    if (packet.size() > 4)
    {
        int recv_timestamp = ((int*)packet.data())[0];
        mostUpdatedTSatPut = recv_timestamp;
        if (firsttimecalling_get)
            return;
        speex_jitter_put((char *)packet.data()+4, packet.size()-4, recv_timestamp);
    }
}

bool SpeexInputProcessor::isSequential() const {
        return true;
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

qint64 SpeexOutputProcessor::readData(char *data, qint64 maxSize) {
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
        //quiet_speex_echo_playback(echo_state, (qint16*)  frame.data());
    }

    QByteArray resultBuffer = outputBuffer.left(maxSize);
    memcpy(data, resultBuffer.data(), resultBuffer.size());

    outputBuffer = outputBuffer.right(outputBuffer.size() - resultBuffer.size());

    return resultBuffer.size();
}

bool SpeexOutputProcessor::isSequential() const {
        return true;
}

void SpeexOutputProcessor::speex_jitter_init(SpeexJitter *jit, void *decoder, int sampling_rate)
{
   jit->dec = decoder;
   speex_decoder_ctl(decoder, SPEEX_GET_FRAME_SIZE, &jit->frame_size);

   jit->packets = jitter_buffer_init(jit->frame_size);
    jit->current_packet = new SpeexBits;
   speex_bits_init(jit->current_packet);
   jit->valid_bits = 0;
}

void SpeexOutputProcessor::speex_jitter_destroy()
{
   jitter_buffer_destroy(jitter.packets);
   speex_bits_destroy(jitter.current_packet);
}

void SpeexOutputProcessor::speex_jitter_put(char *packet, int len, int timestamp)
{
   JitterBufferPacket p;
   p.data = packet;
   p.len = len;
   p.timestamp = timestamp;
   p.span = jitter.frame_size;
   jitter_buffer_put(jitter.packets, &p);
}

void SpeexOutputProcessor::speex_jitter_get(spx_int16_t *out, int *current_timestamp)
{
   int i;
   int ret;
   spx_int32_t activity;
   int bufferCount = 0;
   JitterBufferPacket packet;
   char data[FRAME_SIZE * ECHOTAILSIZE * 10];
   packet.data = data;
   packet.len = FRAME_SIZE * ECHOTAILSIZE * 10;

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

int SpeexOutputProcessor::speex_jitter_get_pointer_timestamp()
{
   return jitter_buffer_get_pointer_timestamp(jitter.packets);
}
