#include "SpeexProcessor.h"

#include <speex/speex.h>
#include <speex/speex_preprocess.h>

using namespace QtSpeex;

SpeexProcessor::SpeexProcessor(int quality, bool wideband, unsigned echo_filter_length, QObject *parent) : QIODevice(parent) {
	bits = new SpeexBits;
	speex_bits_init(bits);

	enc_state = speex_encoder_init(wideband ? &speex_wb_mode : &speex_nb_mode);
	speex_encoder_ctl(enc_state, SPEEX_SET_QUALITY, &quality);
	speex_encoder_ctl(enc_state, SPEEX_GET_FRAME_SIZE, &frame_size);

	dec_state = speex_decoder_init(wideband ? &speex_wb_mode : &speex_nb_mode);

	unsigned rate = wideband ? 16000 : 8000;

	preprocessor = speex_preprocess_state_init(frame_size, rate);

	int state = 1; // on
        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_AGC, &state);
//        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_VAD, &state);
//        speex_preprocess_ctl(preprocessor, SPEEX_PREPROCESS_SET_DENOISE, &state);
//        speex_encoder_ctl(enc_state, SPEEX_SET_VBR, &state);
//        speex_encoder_ctl(enc_state, SPEEX_SET_DTX, &state);
}

SpeexProcessor::~SpeexProcessor() {
	speex_preprocess_state_destroy(preprocessor);

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

	while(inputBuffer.size() > frame_size * sizeof(qint16)) {
		QByteArray frame = inputBuffer.left(frame_size * sizeof(qint16));
		speex_preprocess_run(preprocessor, (qint16*) frame.data());

		speex_bits_reset(bits);
		speex_encode_int(enc_state, (qint16*) frame.constData(), bits);

		QByteArray networkFrame;
		networkFrame.resize(speex_bits_nbytes(bits));
		speex_bits_write(bits, networkFrame.data(), networkFrame.size());

		outputNetworkBuffer.append(networkFrame);

		inputBuffer = inputBuffer.right(inputBuffer.size() - frame_size * sizeof(qint16));

		emit networkPacketReady();
	}

	return maxSize;
}

qint64 SpeexProcessor::readData(char *data, qint64 maxSize) {
	if(outputBuffer.size() > 0 || !inputNetworkBuffer.empty()) {
		while(outputBuffer.size() < maxSize && !inputNetworkBuffer.empty()) {
			QByteArray packet = inputNetworkBuffer.takeFirst();

			QByteArray frame;
			frame.resize(frame_size * sizeof(qint16));

			speex_bits_read_from(bits, packet.data(), packet.size());
			speex_decode_int(dec_state, bits, (qint16*)  frame.data());

			outputBuffer += frame;
		}
	} else {
		// perform extrapolation
		while(outputBuffer.size() < maxSize) {
			QByteArray frame;
			frame.resize(frame_size * sizeof(qint16));

			speex_decode_int(dec_state, NULL, (qint16*) frame.data());

			outputBuffer += frame;
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
