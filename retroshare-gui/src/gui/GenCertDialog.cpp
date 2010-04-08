/****************************************************************
 *  RetroShare is distributed under the following license:
 *
 *  Copyright (C) 2006,  crypton
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, 
 *  Boston, MA  02110-1301, USA.
 ****************************************************************/

#include <rshare.h>
#include <rsiface/rsinit.h>
#include "GenCertDialog.h"
#include "InfoDialog.h"
#include "gui/settings/rsharesettings.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QMovie>
#include <time.h>


/* Define the format used for displaying the date and time */
#define DATETIME_FMT  "MMM dd hh:mm:ss"


/** Default constructor */
GenCertDialog::GenCertDialog(QWidget *parent, Qt::WFlags flags)
  : QDialog(parent, flags)
{
  /* Invoke Qt Designer generated QObject setup routine */
  ui.setupUi(this);

  connect(ui.new_gpg_key_checkbox, SIGNAL(clicked()), this, SLOT(newGPGKeyGenUiSetup()));
  
  connect(ui.genButton, SIGNAL(clicked()), this, SLOT(genPerson()));
  connect(ui.infopushButton,SIGNAL(clicked()), this, SLOT(infodlg()));
  //connect(ui.selectButton, SIGNAL(clicked()), this, SLOT(selectFriend()));
  //connect(ui.friendBox, SIGNAL(stateChanged(int)), this, SLOT(checkChanged(int)));

  //ui.genName->setFocus(Qt::OtherFocusReason);

    /* get all available pgp private certificates....
     * mark last one as default.
     */
    std::cerr << "Finding PGPUsers" << std::endl;

    std::list<std::string> pgpIds;
    std::list<std::string>::iterator it;
    bool foundGPGKeys = false;
    if (RsInit::GetPGPLogins(pgpIds)) {
            for(it = pgpIds.begin(); it != pgpIds.end(); it++)
            {
                    const QVariant & userData = QVariant(QString::fromStdString(*it));
                    std::string name, email;
                    RsInit::GetPGPLoginDetails(*it, name, email);
                    std::cerr << "Adding PGPUser: " << name << " id: " << *it << std::endl;
						  QString gid = QString::fromStdString(*it).right(8) ;
                    ui.genPGPuser->addItem(QString::fromStdString(name + " <" + email + "> (")+gid+")", userData);
                    foundGPGKeys = true;
            }
    }

    if (foundGPGKeys) {
        ui.no_gpg_key_label->hide();
        ui.new_gpg_key_checkbox->setChecked(false);
        genNewGPGKey = false;
    } else {
        ui.no_gpg_key_label->show();
        ui.new_gpg_key_checkbox->setChecked(true);
        ui.new_gpg_key_checkbox->hide();
        genNewGPGKey = true;
    }
    newGPGKeyGenUiSetup();
}

/** Destructor. */
//GenCertDialog::~GenCertDialog()
//{
//}


/** 
 Overloads the default show() slot so we can set opacity*/

void GenCertDialog::show()
{
  //loadSettings();
  if(!this->isVisible()) {
    QWidget::show();

  }
}

void GenCertDialog::closeEvent (QCloseEvent * event)
{


 QDialog::closeEvent(event);
}

void GenCertDialog::closeinfodlg()
{
	close();
}

void GenCertDialog::newGPGKeyGenUiSetup() {
    if (ui.new_gpg_key_checkbox->isChecked()) {
        genNewGPGKey = true;
        ui.name_label->show();
        ui.name_input->show();
        ui.email_label->show();
        ui.email_input->show();
        ui.password_label->show();
        ui.password_input->show();
        ui.genPGPuserlabel->hide();
        ui.genPGPuser->hide();
    } else {
        genNewGPGKey = false;
        ui.name_label->hide();
        ui.name_input->hide();
        ui.email_label->hide();
        ui.email_input->hide();
        ui.password_label->hide();
        ui.password_input->hide();
        ui.genPGPuserlabel->show();
        ui.genPGPuser->show();
    }
}
void GenCertDialog::genPerson()
{

	/* Check the data from the GUI. */
        std::string genLoc  = ui.location_input->text().toStdString();
        std::string PGPId;

        if (!genNewGPGKey) {
            if (ui.location_input->text().length() < 3) {
                    /* Message Dialog */
                    QMessageBox::StandardButton sb = QMessageBox::warning ( NULL,
                                    tr("Generate GPG key Failure"),
                                    tr("Location field is required with a minimum of 3 characters"),
                                      QMessageBox::Ok);
                    return;
            }
            int pgpidx = ui.genPGPuser->currentIndex();
            if (pgpidx < 0)
            {
                    /* Message Dialog */
                    QMessageBox::StandardButton sb = QMessageBox::warning ( NULL,
                                    "Generate ID Failure",
                                    "Missing PGP Certificate",
                                      QMessageBox::Ok);
                    return;
            }
            QVariant data = ui.genPGPuser->itemData(pgpidx);
            PGPId = (data.toString()).toStdString();
        } else {
            if (ui.password_input->text().length() < 3 || ui.name_input->text().length() < 3
                || ui.email_input->text().length() < 3 || ui.location_label->text().length() < 3) {
                    /* Message Dialog */
                    QMessageBox::StandardButton sb = QMessageBox::warning ( NULL,
                                    tr("Generate GPG key Failure"),
                                    tr("All fields are required with a minimum of 3 characters"),
                                      QMessageBox::Ok);
                    return;
            }
            //generate a new gpg key
            std::string err_string;
            ui.no_gpg_key_label->setText(tr("Generating new GPG key, please be patient. Fill in your GPG password when asked."));
            ui.no_gpg_key_label->show();
//            QMovie *movie = new QMovie(":/images/loader/progress.gif");
//            ui.progress_label->setMovie(movie);
//            movie->start();
//            movie->setSpeed(100); // 2x speed
            ui.new_gpg_key_checkbox->hide();
            ui.name_label->hide();
            ui.name_input->hide();
            ui.email_label->hide();
            ui.email_input->hide();
            ui.password_label->hide();
            ui.password_input->hide();
            ui.genPGPuserlabel->hide();
            ui.genPGPuser->hide();
            ui.location_label->hide();
            ui.location_input->hide();
            ui.infopushButton->hide();
            ui.genButton->hide();
            ui.label_location2->hide();
            QMessageBox::StandardButton info = QMessageBox::information( NULL,
                            "Generating GPG key",
                            "This process can take some time, please be patient after pressing the OK button",
                              QMessageBox::Ok);
            //info->
            RsInit::GeneratePGPCertificate(ui.name_input->text().toStdString(), ui.email_input->text().toStdString(), ui.password_input->text().toStdString(), PGPId, err_string);
        }


	//generate a random ssl password
	std::cerr << " generating sslPasswd." << std::endl;
	qsrand(time(NULL));
	std::string sslPasswd = "";
	const int PWD_LEN = RsInit::getSslPwdLen();

	for( int i = 0 ; i < PWD_LEN ; ++i )
	{
	    int iNumber;
	    iNumber = qrand()%25 + 65;
	    sslPasswd += (char)iNumber;
	}

	/* Initialise the PGP user first */
	RsInit::SelectGPGAccount(PGPId);
        //RsInit::LoadGPGPassword(PGPpasswd);

	std::string sslId;
        std::cerr << "GenCertDialog::genPerson() Generating SSL cert with gpg id : " << PGPId << std::endl;
        std::string err;
        bool okGen = RsInit::GenerateSSLCertificate(PGPId, "", genLoc, "", sslPasswd, sslId, err);

	if (okGen)
	{
		/* complete the process */
		RsInit::LoadPassword(sslId, sslPasswd);
		loadCertificates();
	}
	else
	{
		/* Message Dialog */
		QMessageBox::StandardButton sb = QMessageBox::warning ( NULL,
	                        "Generate ID Failure",
				"Failed to Generate your new Certificate, maybe PGP password is wrong !",
			          QMessageBox::Ok);
	}
}





void GenCertDialog::selectFriend()
{

#if 0
	/* still need to find home (first) */

	QString fileName = QFileDialog::getOpenFileName(this, tr("Select Trusted Friend"), "",
                                             tr("Certificates (*.pqi *.pem)"));

	std::string fname, userName;
	fname = fileName.toStdString();
	if (RsInit::ValidateTrustedUser(fname, userName))
	{
		ui.genFriend -> setText(QString::fromStdString(userName));
	}
	else
	{
		ui.genFriend -> setText("<Invalid Selected>");
	}
#endif

}


void GenCertDialog::checkChanged(int i)
{

#if 0
	if (i)
	{
		selectFriend();
	}
	else
	{
		/* invalidate selection */
		std::string fname = "";
		std::string userName = "";
		RsInit::ValidateTrustedUser(fname, userName);
		ui.genFriend -> setText("<None Selected>");
	}
#endif

}


void GenCertDialog::loadCertificates()
{
	bool autoSave = false; 
	/* Final stage of loading */
	if (RsInit::LoadCertificates(autoSave))
	{
		close();
	}
	else
	{
		/* some error msg */
		QMessageBox::StandardButton sb = QMessageBox::warning ( NULL,
	                        "Generate ID Failure",
			        "Failed to Load your new Certificate!",
			          QMessageBox::Ok);
	}
}

void GenCertDialog::infodlg()
{
    static InfoDialog *infodialog = new InfoDialog();
    infodialog->show();
}
