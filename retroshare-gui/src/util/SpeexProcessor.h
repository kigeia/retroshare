/*
 * QtSpeex - Speex bindings for Qt
 * Copyright (C) 2010 Peter Zotov
 */

#ifndef SPEEXPROCESSOR_H
#define SPEEXPROCESSOR_H

#include <QIODevice>
#include <QByteArray>
#include <QList>

#include <speex/speex_preprocess.h>
#include <speex/speex_echo.h>

class SpeexBits;

namespace QtSpeex {
	class SpeexProcessor : public QIODevice {
		Q_OBJECT

	public:
		SpeexProcessor(int quality = 5,  bool wideband = true, unsigned echo_filter_length = 100, QObject* parent = 0);
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

		QByteArray inputBuffer, outputBuffer;
		QList<QByteArray> inputNetworkBuffer, outputNetworkBuffer;
	};
}

#endif // SPEEXPROCESSOR_H
