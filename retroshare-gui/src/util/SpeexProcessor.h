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

#include <speex/speex_preprocess.h>
#include <speex/speex_echo.h>
#include <speex/speex_jitter.h>

#define ECHOTAILSIZE  40
#define SAMPLING_RATE 16000
#define FRAME_SIZE 320

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
	class SpeexProcessor : public QIODevice {
		Q_OBJECT

	public:
                SpeexProcessor(QObject* parent = 0);
		virtual ~SpeexProcessor();

		bool hasPendingPackets();
		QByteArray getNetworkPacket();

		void putNetworkPacket(QByteArray packet);

	signals:
		void networkPacketReady();

	protected:
		virtual qint64 readData(char *data, qint64 maxSize);
		virtual qint64 writeData(const char *data, qint64 maxSize);
		virtual bool isSequential() const;

	private:
		void* enc_state;
		void* dec_state;
		unsigned frame_size;
		SpeexBits* bits;

                SpeexPreprocessState* preprocessor;
                SpeexEchoState       *echo_state;

                short psSpeaker[320 * ECHOTAILSIZE * 10];//use for echo cancelation (speaker output). may not be usefull if using speex_echo_capture instead of speex_echo_cancellation
                JitterBuffer *jitterBuffer ;
                QByteArray inputBuffer, outputBuffer;
		QList<QByteArray> inputNetworkBuffer, outputNetworkBuffer;

                SpeexJitter* jitter;

                void speex_jitter_init(void *decoder, int sampling_rate);
                void speex_jitter_destroy();
                void speex_jitter_put(char *packet, int len, int timestamp);
                void speex_jitter_get(spx_int16_t *out, int *current_timestamp);
                int speex_jitter_get_pointer_timestamp();
                short                 pcm[FRAME_SIZE], pcm2[FRAME_SIZE];//buffer for encoding internals
                short                 dec_pcm[FRAME_SIZE];//buffer for dec internals
                short                 ps[FRAME_SIZE];//buffer for dec internals
        };
}

#endif // SPEEXPROCESSOR_H
