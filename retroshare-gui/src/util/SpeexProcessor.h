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
#include <QHash>

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
   int mostUpdatedTSatPut;           /**< timestamp of the last packet put */
   bool firsttimecalling_get;
} SpeexJitter;

namespace QtSpeex {
        class SpeexInputProcessor : public QIODevice {
		Q_OBJECT

	public:
                float dPeakSpeaker, dPeakSignal, dMaxMic, dPeakMic, dPeakCleanMic, dVoiceAcivityLevel;
                float fSpeechProb;
                SpeexInputProcessor(QObject* parent = 0);
                virtual ~SpeexInputProcessor();

		bool hasPendingPackets();
                QByteArray getNetworkPacket();

                int iMaxBitRate;
                int iRealTimeBitrate;
                bool bPreviousVoice;
                void setEchoState(SpeexEchoState*);//use an echo_state from an output speex processor

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
                short * psClean; //temp buffer for audio sampling after echo cleaning (if enabled)

                QByteArray inputBuffer;
                QList<QByteArray> outputNetworkBuffer;

                short                 pcm[FRAME_SIZE], pcm2[FRAME_SIZE];//buffer for encoding internals
        };


        class SpeexOutputProcessor : public QIODevice {
                Q_OBJECT

        public:
                SpeexOutputProcessor(QObject* parent = 0);
                virtual ~SpeexOutputProcessor();

                void putNetworkPacket(QString name, QByteArray packet);
                SpeexEchoState* initEchoState();

        protected:
                virtual qint64 readData(char *data, qint64 maxSize);
                virtual qint64 writeData(const char *data, qint64 maxSize) {return 0;} //not used for output processor
                virtual bool isSequential() const;

        private:
                SpeexEchoState       *echo_state;

                QByteArray outputBuffer;
                QList<QByteArray> inputNetworkBuffer;

                QHash<QString, SpeexJitter*> userJitterHash;

                //SpeexJitter jitter;

                void speex_jitter_init(SpeexJitter *jit, void *decoder, int sampling_rate);
                void speex_jitter_destroy(SpeexJitter jitter);
                void speex_jitter_put(SpeexJitter jitter, char *packet, int len, int timestamp);
                void speex_jitter_get(SpeexJitter jitter, spx_int16_t *out, int *current_timestamp);
                int speex_jitter_get_pointer_timestamp(SpeexJitter jitter);
        };
    }

#endif // SPEEXPROCESSOR_H
