/*
 * QtSpeex - Speex bindings for Qt
 * Copyright (C) 2010 Peter Zotov
 */

#ifndef SPEEXPROCESSOR_H
#define SPEEXPROCESSOR_H

#include <iostream>

#include <QIODevice>
#include <QByteArray>
#include <QList>
#include <QMutex>
#include <QAudioInput>
#include <QAudioOutput>

#include <speex/speex_preprocess.h>
#include <speex/speex_echo.h>
#include <speex/speex_jitter.h>

#define ECHOTAILSIZE  30
#define SAMPLING_RATE 16000 //must be the same as the speex setted mode (speex_wb_mode)
#define FRAME_SIZE 320 //must be the same as the speex setted mode (speex_wb_mode)

class SpeexBits;

/** Speex jitter-buffer state. Never use it directly! */
typedef struct SpeexJitter {
   SpeexBits *current_packet;         /**< Current Speex packet */
   int valid_bits;                   /**< True if Speex bits are valid */
   JitterBuffer *packets;            /**< Generic jitter buffer state */
   void *dec;                        /**< Pointer to Speex decoder */
   spx_int32_t frame_size;           /**< Frame size of Speex decoder */
} SpeexJitter;

namespace QtSpeex {
        class SpeexInputProcessor : public QIODevice {
		Q_OBJECT

	public:
                float dPeakSpeaker, dPeakSignal, dMaxMic, dPeakMic, dPeakCleanMic, dVoiceAcivityLevel;
                float fSpeechProb;
                SpeexInputProcessor(QObject* parent = 0);//upon construction, it automatically connects to an input audio device
                virtual ~SpeexInputProcessor();

		bool hasPendingPackets();
                QByteArray getNetworkPacket();

                int iMaxBitRate;
                int iRealTimeBitrate;
                bool bPreviousVoice;
                void stopAudioInputDevice();
                void startAudioInputDevice();
                QAudioInput *getAudioInputDevice();



	signals:
		void networkPacketReady();

	protected:
                virtual qint64 readData(char *data, qint64 maxSize) {return false;} //not used for input processor
		virtual qint64 writeData(const char *data, qint64 maxSize);
                virtual bool isSequential() const;

        private:
                int iFrameCounter;
                int iSilentFrames;
                int iHoldFrames;

                QMutex qmSpeex;
                void* enc_state;
                SpeexBits* enc_bits;
                int send_timestamp; //set at the encode time for the jitter buffer of the reciever

                bool bResetProcessor;

                SpeexPreprocessState* preprocessor;
                SpeexEchoState       *echo_state;

                QByteArray inputBuffer;
                QList<QByteArray> outputNetworkBuffer;

                short                 pcm[FRAME_SIZE], pcm2[FRAME_SIZE];//buffer for encoding internals
        };


        class SpeexOutputProcessor : public QIODevice {
                Q_OBJECT

        public:
                SpeexOutputProcessor(QObject* parent = 0);
                virtual ~SpeexOutputProcessor();

                void putNetworkPacket(QByteArray packet);

        protected:
                virtual qint64 readData(char *data, qint64 maxSize);
                virtual qint64 writeData(const char *data, qint64 maxSize) {return 0;} //not used for output processor
                virtual bool isSequential() const;

        private:
                QAudioOutput *qAudioOutput;
                void* dec_state;
                SpeexBits* dec_bits;
                int  mostUpdatedTSatPut; //use for the decoder jitter
                bool firsttimecalling_get;

                SpeexEchoState       *echo_state;

                JitterBuffer *jitterBuffer ;
                QByteArray outputBuffer;
                QList<QByteArray> inputNetworkBuffer;

                SpeexJitter jitter;

                void speex_jitter_init(SpeexJitter *jit, void *decoder, int sampling_rate);
                void speex_jitter_destroy();
                void speex_jitter_put(char *packet, int len, int timestamp);
                void speex_jitter_get(spx_int16_t *out, int *current_timestamp);
                int speex_jitter_get_pointer_timestamp();
                short                 dec_pcm[FRAME_SIZE];//buffer for dec internals
                short                 ps[FRAME_SIZE];//buffer for dec internals
        };
    }

// Initialisation des singletons à NULL
//QtSpeex::SpeexInputProcessor *QtSpeex::SpeexInputProcessor::_singleton = NULL;
#endif // SPEEXPROCESSOR_H
