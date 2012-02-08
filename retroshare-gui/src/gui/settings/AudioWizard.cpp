/* Copyright (C) 2005-2010, Thorvald Natvig <thorvald@natvig.com>

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

#include "AudioWizard.h"
//#include "AudioInput.h"
//#include "Global.h"
//#include "Settings.h"
//#include "Log.h"
//#include "MainWindow.h"
#include "gui/settings/rsharesettings.h"
#include "util/audiodevicehelper.h"

#define iroundf(x) ( static_cast<int>(x) )

AudioWizard::~AudioWizard()
{
    if (inputDevice) {
        inputDevice->stop();
    }
    if (outputDevice) {
        outputDevice->stop();
    }
}

AudioWizard::AudioWizard(QWidget *p) : QWizard(p) {
	bInit = true;
	bLastActive = false;
        //g.bInAudioWizard = true;

	ticker = new QTimer(this);
	ticker->setObjectName(QLatin1String("Ticker"));

        setupUi(this);
        inputProcessor = NULL;
        inputDevice = NULL;
        outputProcessor = NULL;
        outputDevice = NULL;

	// Done
        //qcbUsage->setChecked(g.s.bUsage);

	// Device
        /*if (AudioInputRegistrar::qmNew) {
		foreach(AudioInputRegistrar *air, *AudioInputRegistrar::qmNew) {
			qcbInput->addItem(air->name);
			if (air->name == AudioInputRegistrar::current) {
				qcbInput->setCurrentIndex(qcbInput->count() - 1);
				qcbEcho->setEnabled(air->canEcho(qcbOutput->currentText()));
			}
			QList<audioDevice> ql= air->getDeviceChoices();
		}
        }
        if (qcbInput->count() < 2) {
		qcbInput->setEnabled(false);
        }*/

        //qcbEcho->setChecked(g.s.bEcho);

        /*if (AudioOutputRegistrar::qmNew) {
		foreach(AudioOutputRegistrar *aor, *AudioOutputRegistrar::qmNew) {
			qcbOutput->addItem(aor->name);
			if (aor->name == AudioOutputRegistrar::current) {
				qcbOutput->setCurrentIndex(qcbOutput->count() - 1);
				bDelay = aor->usesOutputDelay();
				qcbAttenuateOthers->setEnabled(aor->canMuteOthers());
			}
			QList<audioDevice> ql= aor->getDeviceChoices();
		}
        }

        if (qcbOutput->count() < 2) {
		qcbOutput->setEnabled(false);
        }*/

        //qcbHighContrast->setChecked(g.s.bHighContrast);
        //on_qcbHighContrast_clicked(g.s.bHighContrast);
#ifdef Q_OS_WIN
	// On windows we can autodetect this
	qcbHighContrast->setVisible(false);
#endif

        /*// Settings
	if (g.s.iQuality == 16000 && g.s.iFramesPerPacket == 6)
		qrbQualityLow->setChecked(true);
	else if (g.s.iQuality == 40000 && g.s.iFramesPerPacket == 2)
		qrbQualityBalanced->setChecked(true);
	else if (g.s.iQuality == 72000 && g.s.iFramesPerPacket == 1)
		qrbQualityUltra->setChecked(true);
	else
		qrbQualityCustom->setChecked(true);
        */
        /*quint32 iMessage = Settings::LogNone;
	for (int i = Log::firstMsgType;i <= Log::lastMsgType; ++i) {
		iMessage |= (g.s.qmMessages[i] & (Settings::LogSoundfile | Settings::LogTTS));
	}

	if (iMessage == Settings::LogTTS && g.s.bTTS)
		qrbNotificationTTS->setChecked(true);
	else if (iMessage == Settings::LogSoundfile)
		qrbNotificationSounds->setChecked(true);
	else // If we find mixed message types or only tts with main tts disable assume custom
		qrbNotificationCustom->setChecked(true);

	qrbNotificationCustom->setVisible(qrbNotificationCustom->isChecked());

	qrbQualityCustom->setVisible(qrbQualityCustom->isChecked());
	qlQualityCustom->setVisible(qrbQualityCustom->isChecked());

	qcbPositional->setChecked(g.s.bPositionalAudio);
	qcbAttenuateOthers->setChecked(g.s.bAttenuateOthers);

	on_qcbInput_activated(qcbInput->currentIndex());
	on_qcbOutput_activated(qcbOutput->currentIndex());
        */
        abAmplify = new AudioBar(this);
	abAmplify->qcBelow = Qt::blue;
	abAmplify->qcInside = Qt::green;
	abAmplify->qcAbove = Qt::red;

        verticalLayout_3->addWidget(abAmplify);

	// Trigger
        /*foreach(const Shortcut &s, g.s.qlShortcuts) {
		if (s.iIndex == g.mw->gsPushTalk->idx) {
			skwPTT->setShortcut(s.qlButtons);
			break;
		}
        }*/

        if (Settings->getVoipATransmit() == RshareSettings::AudioTransmitPushToTalk)
		qrPTT->setChecked(true);
        else if (Settings->getVoipATransmit() == RshareSettings::AudioTransmitVAD)
                qrVAD->setChecked(true);
	else
                qrContinuous->setChecked(true);

        abVAD = new AudioBar(this);
	abVAD->qcBelow = Qt::red;
	abVAD->qcInside = Qt::yellow;
	abVAD->qcAbove = Qt::green;

        qsTransmitMin->setValue(Settings->getVoipfVADmin());
        qsTransmitMax->setValue(Settings->getVoipfVADmax());

        verticalLayout_6->addWidget(abVAD);

	// Volume
        qsMaxAmp->setValue(Settings->getVoipiMinLoudness());

	setOption(QWizard::NoCancelButton, false);
	resize(700, 500);

        updateTriggerWidgets(qrVAD->isChecked());

	bTransmitChanged = false;

	iMaxPeak = 0;
	iTicks = 0;

	qpTalkingOn = QPixmap::fromImage(QImage(QLatin1String("skin:talking_on.svg")).scaled(64,64));
	qpTalkingOff = QPixmap::fromImage(QImage(QLatin1String("skin:talking_off.svg")).scaled(64,64));

	bInit = false;

	connect(this, SIGNAL(currentIdChanged(int)), this, SLOT(showPage(int)));

	ticker->setSingleShot(false);
	ticker->start(20);
        connect( ticker, SIGNAL( timeout ( ) ), this, SLOT( on_Ticker_timeout() ) );
}

/*bool AudioWizard::eventFilter(QObject *obj, QEvent *evt) {
	if ((evt->type() == QEvent::MouseButtonPress) ||
	        (evt->type() == QEvent::MouseMove)) {
		QMouseEvent *qme = dynamic_cast<QMouseEvent *>(evt);
		if (qme) {
			if (qme->buttons() & Qt::LeftButton) {
				QPointF qpf = qgvView->mapToScene(qme->pos());
				fX = static_cast<float>(qpf.x());
				fY = static_cast<float>(qpf.y());
			}
		}
	}
	return QWizard::eventFilter(obj, evt);
}*/



void AudioWizard::on_qsMaxAmp_valueChanged(int v) {
        Settings->setVoipiMinLoudness(qMin(v, 30000));
}

void AudioWizard::on_Ticker_timeout() {
        if (!inputProcessor) {
            inputProcessor = new QtSpeex::SpeexInputProcessor();
            inputProcessor->open(QIODevice::WriteOnly | QIODevice::Unbuffered);

            if (!inputDevice) {
                inputDevice = AudioDeviceHelper::getPreferedInputDevice();
            }
            inputDevice->start(inputProcessor);
            connect(inputProcessor, SIGNAL(networkPacketReady()), this, SLOT(loopAudio()));
        }

        if (!outputProcessor) {
            outputProcessor = new QtSpeex::SpeexOutputProcessor();
            outputProcessor->open(QIODevice::ReadOnly | QIODevice::Unbuffered);

            if (!outputDevice) {
                outputDevice = AudioDeviceHelper::getPreferedOutputDevice();
            }
            outputDevice->start(outputProcessor);
        }

        abVAD->iBelow = qsTransmitMin->value();
        abVAD->iAbove = qsTransmitMax->value();
        Settings->setVoipfVADmin(qsTransmitMin->value());
        Settings->setVoipfVADmax(qsTransmitMax->value());

        abVAD->iValue = iroundf(inputProcessor->dVoiceAcivityLevel * 32767.0f + 0.5f);

        abVAD->update();

        int iPeak = inputProcessor->dMaxMic;

	if (iTicks++ >= 50) {
		iMaxPeak = 0;
		iTicks = 0;
	}
	if (iPeak > iMaxPeak)
		iMaxPeak = iPeak;

	abAmplify->iBelow = qsMaxAmp->value();
	abAmplify->iValue = iPeak;
	abAmplify->iPeak = iMaxPeak;
	abAmplify->update();

        bool active = inputProcessor->bPreviousVoice;
	if (active != bLastActive) {
		bLastActive = active;
		qlTalkIcon->setPixmap(active ? qpTalkingOn : qpTalkingOff);
	}
}

void AudioWizard::loopAudio() {
    if (outputDevice && outputDevice->error() != QAudio::NoError) {
        //TODO : find a way to restart output device, but there is a pulseaudio locks that prevents it here
        std::cerr << "Restarting output (and input) device. Error before reset " << outputDevice->error() << " buffer size : " << outputDevice->bufferSize() << std::endl;
        inputDevice->stop();
        outputDevice->stop();
        inputDevice->start(inputProcessor);
        outputDevice->start(outputProcessor);
        std::cerr << "Output device restarted." << std::endl;
    }
    while(outputProcessor && inputProcessor && inputProcessor->hasPendingPackets()) {
        std::cerr << "Processing packet." << std::endl;
        outputProcessor->putNetworkPacket(inputProcessor->getNetworkPacket());
    }
}

void AudioWizard::on_qsTransmitMax_valueChanged(int v) {
	if (! bInit) {
                Settings->setVoipfVADmax(v);
	}
}

void AudioWizard::on_qsTransmitMin_valueChanged(int v) {
        if (! bInit) {
                Settings->setVoipfVADmin(v);
        }
}

void AudioWizard::on_qrVAD_clicked(bool on) {
	if (on) {
                Settings->setVoipATransmit(RshareSettings::AudioTransmitVAD);
                updateTriggerWidgets(true);
		bTransmitChanged = true;
	}
}

void AudioWizard::on_qrPTT_clicked(bool on) {
	if (on) {
                Settings->setVoipATransmit(RshareSettings::AudioTransmitPushToTalk);
                updateTriggerWidgets(false);
		bTransmitChanged = true;
	}
}

void AudioWizard::on_qrContinuous_clicked(bool on) {
        if (on) {
                Settings->setVoipATransmit(RshareSettings::AudioTransmitContinous);
                updateTriggerWidgets(false);
                bTransmitChanged = true;
        }
}
/*void AudioWizard::on_skwPTT_keySet(bool valid, bool last) {
	if (valid)
		qrPTT->setChecked(true);
	else if (qrPTT->isChecked())
		qrAmplitude->setChecked(true);
	updateTriggerWidgets(valid);
	bTransmitChanged = true;

	if (last) {

		const QList<QVariant> &buttons = skwPTT->getShortcut();
		QList<Shortcut> ql;
		bool found = false;
		foreach(Shortcut s, g.s.qlShortcuts) {
			if (s.iIndex == g.mw->gsPushTalk->idx) {
				if (buttons.isEmpty())
					continue;
				else if (! found) {
					s.qlButtons = buttons;
					found = true;
				}
			}
			ql << s;
		}
		if (! found && ! buttons.isEmpty()) {
			Shortcut s;
			s.iIndex = g.mw->gsPushTalk->idx;
			s.bSuppress = false;
			s.qlButtons = buttons;
			ql << s;
		}
		g.s.qlShortcuts = ql;
		GlobalShortcutEngine::engine->bNeedRemap = true;
		GlobalShortcutEngine::engine->needRemap();
	}
}*/


void AudioWizard::updateTriggerWidgets(bool vad_on) {
        if (!vad_on)
            qwVAD->hide();
        else
            qwVAD->show();
}

void AudioWizard::on_qcbHighContrast_clicked(bool on) {
        abAmplify->highContrast = on;
        abVAD->highContrast = on;
}
