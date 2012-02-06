/* Copyright (C) 2005-2010, Thorvald Natvig <thorvald@natvig.com>
   Copyright (C) 2008, Andreas Messer <andi@bupfen.de>

   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   - Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
   - Neither the name of the Mumble Developers nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//#include "AudioInput.h"
//#include "AudioOutput.h"
#include "AudioStats.h"
#include "AudioInputConfig.h"
//#include "Global.h"
//#include "NetworkConfig.h"
#include "rsharesettings.h"
#include "util/audiodevicehelper.h"

#define iroundf(x) ( static_cast<int>(x) )

/*void AudioInputDialog::hideEvent(QHideEvent *) {
	qtTick->stop();
}

void AudioInputDialog::showEvent(QShowEvent *) {
	qtTick->start(20);
}*/

/** Constructor */
AudioInputConfig::AudioInputConfig(QWidget * parent, Qt::WFlags flags)
    : ConfigPage(parent, flags)
{
    /* Invoke the Qt Designer generated object setup routine */
    ui.setupUi(this);

    /* Hide platform specific features */
#ifdef Q_WS_WIN

#endif
}
/** Loads the settings for this page */
void
AudioInputConfig::load()
{
    qtTick = new QTimer(this);
    qtTick->setObjectName(QLatin1String("Tick"));
    qtTick->start(20);
    /*if (AudioInputRegistrar::qmNew) {
            QList<QString> keys = AudioInputRegistrar::qmNew->keys();
            foreach(QString key, keys) {
                    qcbSystem->addItem(key);
            }
    }
    qcbSystem->setEnabled(qcbSystem->count() > 1);*/

    ui.qcbTransmit->addItem(tr("Continuous"), RshareSettings::AudioTransmitContinous);
    ui.qcbTransmit->addItem(tr("Voice Activity"), RshareSettings::AudioTransmitVAD);
    ui.qcbTransmit->addItem(tr("Push To Talk"), RshareSettings::AudioTransmitPushToTalk);

    abSpeech = new AudioBar();
    abSpeech->qcBelow = Qt::red;
    abSpeech->qcInside = Qt::yellow;
    abSpeech->qcAbove = Qt::green;

    //ui.qwVAD->

    ui.qcbDevice->view()->setTextElideMode(Qt::ElideRight);

    //ui.on_qcbPushClick_clicked(g.s.bPushClick);
    //ui.on_Tick_timeout();
    loadSettings();
    on_Tick_timeout();
}


void AudioInputConfig::loadSettings() {
	int i;
	QList<QString> keys;

        /*if (AudioInputRegistrar::qmNew)
		keys=AudioInputRegistrar::qmNew->keys();
	else
		keys.clear();
	i=keys.indexOf(AudioInputRegistrar::current);
	if (i >= 0)
                loadComboBox(qcbSystem, i);

        loadCheckBox(qcbExclusive, r.bExclusiveInput);*/

        //qlePushClickPathOn->setText(r.qsPushClickOn);
        //qlePushClickPathOff->setText(r.qsPushClickOff);

        /*loadComboBox(qcbTransmit, r.atTransmit);
	loadSlider(qsTransmitHold, r.iVoiceHold);
	loadSlider(qsTransmitMin, iroundf(r.fVADmin * 32767.0f + 0.5f));
	loadSlider(qsTransmitMax, iroundf(r.fVADmax * 32767.0f + 0.5f));
	loadSlider(qsFrames, (r.iFramesPerPacket == 1) ? 1 : (r.iFramesPerPacket/2 + 1));
        loadSlider(qsDoublePush, iroundf(static_cast<float>(r.uiDoublePush) / 1000.f + 0.5f));*/

        ui.qcbTransmit->setCurrentIndex(Settings->getVoipATransmit());
        ui.qsTransmitHold->setValue(Settings->getVoiceHold());
        ui.qsTransmitMin->setValue(iroundf(Settings->getVoipfVADmin() * 32767.0f + 0.5f));
        ui.qsTransmitMax->setValue(iroundf(Settings->getVoipfVADmax() * 32767.0f + 0.5f));
        //ui.qsDoublePush->setValue(iroundf(static_cast<float>(r.uiDoublePush) / 1000.f + 0.5f));

        if (Settings->getVoipVoiceActivityD() == RshareSettings::VADSourceAmplitude)
                ui.qrbAmplitude->setChecked(true);
	else
                ui.qrbSNR->setChecked(true);

        //loadCheckBox(qcbPushClick, r.bPushClick);
        //loadSlider(qsQuality, r.iQuality);
        if (Settings->getVoipiNoiseSuppress() != 0)
                ui.qsNoise->setValue(-Settings->getVoipiNoiseSuppress());
	else
                ui.qsNoise->setValue(14);

        //loadSlider(qsAmp, 20000 - r.iMinLoudness);
        //loadSlider(qsIdle, r.iIdleTime);

        /*int echo = 0;
	if (r.bEcho)
		echo = r.bEchoMulti ? 2 : 1;

        loadComboBox(qcbEcho, echo);*/
}

bool AudioInputConfig::save(QString &/*errmsg*/) {
        //s.iQuality = qsQuality->value();
        Settings->setVoipiNoiseSuppress((ui.qsNoise->value() == 14) ? 0 : - ui.qsNoise->value());
        Settings->setVoipiMinLoudness(18000 - ui.qsAmp->value() + 2000);
        Settings->setVoiceHold(ui.qsTransmitHold->value());
        Settings->setVoipfVADmin(static_cast<float>(ui.qsTransmitMin->value()) / 32767.0f);
        Settings->setVoipfVADmax(static_cast<float>(ui.qsTransmitMax->value()) / 32767.0f);
        Settings->setVoipVoiceActivityD(ui.qrbSNR->isChecked() ? RshareSettings::VADSourceSignalToNoise : RshareSettings::VADSourceAmplitude);
        /*s.uiDoublePush = qsDoublePush->value() * 1000;*/
        Settings->setVoipATransmit(static_cast<RshareSettings::enumAudioTransmit>(ui.qcbTransmit->currentIndex()));

        /*s.bPushClick = qcbPushClick->isChecked();
	s.qsPushClickOn = qlePushClickPathOn->text();
        s.qsPushClickOff = qlePushClickPathOff->text();*/

        /*s.qsAudioInput = qcbSystem->currentText();
	s.bEcho = qcbEcho->currentIndex() > 0;
	s.bEchoMulti = qcbEcho->currentIndex() == 2;
        s.bExclusiveInput = qcbExclusive->isChecked();*/

        /*if (AudioInputRegistrar::qmNew) {
		AudioInputRegistrar *air = AudioInputRegistrar::qmNew->value(qcbSystem->currentText());
		int idx = qcbDevice->currentIndex();
		if (idx > -1) {
			air->setDeviceChoice(qcbDevice->itemData(idx), s);
		}
        }*/
}

/*bool AudioInputDialog::expert(bool b) {
        qgbInterfaces->setVisible(b);
	qgbAudio->setVisible(b);
	qliFrames->setVisible(b);
	qsFrames->setVisible(b);
	qlFrames->setVisible(b);
	qswTransmit->setVisible(b);
	qliIdle->setVisible(b);
	qsIdle->setVisible(b);
        qlIdle->setVisible(b);
	return true;
}*/


void AudioInputConfig::on_qsTransmitHold_valueChanged(int v) {
	float val = static_cast<float>(v * 10);
	val = val / 1000.0f;
        ui.qlTransmitHold->setText(tr("%1 s").arg(val, 0, 'f', 2));
}

void AudioInputConfig::on_qsNoise_valueChanged(int v) {
	QPalette pal;

	if (v < 15) {
                ui.qlNoise->setText(tr("Off"));
                pal.setColor(ui.qlNoise->foregroundRole(), Qt::red);
	} else {
                ui.qlNoise->setText(tr("-%1 dB").arg(v));
	}
        ui.qlNoise->setPalette(pal);
}

void AudioInputConfig::on_qsAmp_valueChanged(int v) {
	v = 18000 - v + 2000;
	float d = 20000.0f/static_cast<float>(v);
        ui.qlAmp->setText(QString::fromLatin1("%1").arg(d, 0, 'f', 2));
}


void AudioInputConfig::on_qcbTransmit_currentIndexChanged(int v) {
	switch (v) {
		case 0:
                        ui.qswTransmit->setCurrentWidget(ui.qwContinuous);
			break;
		case 1:
                        ui.qswTransmit->setCurrentWidget(ui.qwVAD);
			break;
		case 2:
                        ui.qswTransmit->setCurrentWidget(ui.qwPTT);
			break;
	}
}


void AudioInputConfig::on_Tick_timeout() {
        if (!inputProcessor) {
            inputProcessor = new QtSpeex::SpeexInputProcessor();
            inputProcessor = new QtSpeex::SpeexInputProcessor();
            inputProcessor->open(QIODevice::WriteOnly | QIODevice::Unbuffered);

            if (!inputDevice) {
                inputDevice = AudioDeviceHelper::getPreferedInputDevice();
            }
            inputDevice->start(inputProcessor);
        }

        abSpeech->iBelow = ui.qsTransmitMin->value();
        abSpeech->iAbove = ui.qsTransmitMax->value();

        if (ui.qrbAmplitude->isChecked()) {
                abSpeech->iValue = iroundf((32767.f/96.0f) * (96.0f + inputProcessor->dPeakCleanMic) + 0.5f);
        } else {
                abSpeech->iValue = iroundf(inputProcessor->fSpeechProb * 32767.0f + 0.5f);
        }
        abSpeech->update();
}
