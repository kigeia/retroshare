/****************************************************************
 *  RetroShare is distributed under the following license:
 *
 *  Copyright (C) 2006- 2010 RetroShare Team
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
#include "ShareDialog.h"

#include <retroshare/rsfiles.h>

#include <QContextMenuEvent>
#include <QFileDialog>
#include <QMessageBox>
#include <QComboBox>

/** Default constructor */
ShareDialog::ShareDialog(std::string filename, QWidget *parent, Qt::WFlags flags)
  : QDialog(parent, flags)
{
    /* Invoke Qt Designer generated QObject setup routine */
    ui.setupUi(this);

    connect(ui.browseButton, SIGNAL(clicked( bool ) ), this , SLOT( browseDirectory() ) );
    connect(ui.okButton, SIGNAL(clicked( bool ) ), this , SLOT( addDirectory() ) );
    connect(ui.closeButton, SIGNAL(clicked()), this, SLOT(close()));

    ui.okButton->setEnabled(false);

    if (filename.empty()) {
        ui.networkwideCheckBox->setChecked(true);
    } else {
        /* edit exisiting share */
        std::list<SharedDirInfo> dirs;
        rsFiles->getSharedDirectories(dirs);

        std::list<SharedDirInfo>::const_iterator it;
        for (it = dirs.begin(); it != dirs.end(); it++) {
            if (it->filename == filename) {
                /* fill dialog */
                ui.okButton->setEnabled(true);

                ui.localpath_lineEdit->setText(QString::fromUtf8(it->filename.c_str()));
                ui.localpath_lineEdit->setDisabled(true);
                ui.browseButton->setDisabled(true);
                ui.virtualpath_lineEdit->setText(QString::fromUtf8(it->virtualname.c_str()));

                ui.browsableCheckBox->setChecked(it->shareflags & RS_FILE_HINTS_BROWSABLE);
                ui.networkwideCheckBox->setChecked(it->shareflags & RS_FILE_HINTS_NETWORK_WIDE);
                break;
            }
        }
    }
}

void ShareDialog::browseDirectory()
{
    /* select a dir*/
    QString qdir = QFileDialog::getExistingDirectory(this, tr("Select A Folder To Share"), "", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

    /* add it to the server */
    if (qdir.isEmpty()) {
        ui.okButton->setEnabled(false);
        return;
    }
    ui.okButton->setEnabled(true);
    ui.localpath_lineEdit->setText(qdir);
}

void ShareDialog::addDirectory()
{
    SharedDirInfo sdi ;
    sdi.filename = ui.localpath_lineEdit->text().toUtf8().constData();
    sdi.virtualname = ui.virtualpath_lineEdit->text().toUtf8().constData();

    sdi.shareflags = 0;

    if (ui.browsableCheckBox->isChecked()) {
        sdi.shareflags |= RS_FILE_HINTS_BROWSABLE ;
    }
    if (ui.networkwideCheckBox->isChecked()) {
        sdi.shareflags |= RS_FILE_HINTS_NETWORK_WIDE;
    }

    if (ui.localpath_lineEdit->isEnabled()) {
        /* add new share */
        rsFiles->addSharedDirectory(sdi);
    } else {
        /* edit exisiting share */
        bool found = false;

        std::list<SharedDirInfo> dirs;
        rsFiles->getSharedDirectories(dirs);

        std::list<SharedDirInfo>::iterator it;
        for (it = dirs.begin(); it != dirs.end(); it++) {
            if (it->filename == sdi.filename) {
                found = true;

                if (it->virtualname != sdi.virtualname) {
                    /* virtual name changed, remove shared directory and add it again */
                    rsFiles->removeSharedDirectory(it->filename);
                    rsFiles->addSharedDirectory(sdi);
                    break;
                }
                if (it->shareflags ^ sdi.shareflags) {
                    /* modifies the flags */
                    it->shareflags = sdi.shareflags;
                    rsFiles->updateShareFlags(*it);
                    break;
                }

                /* nothing changed */
                break;
            }
        }

        if (found == false) {
            /* not modified, add share directory instead */
            rsFiles->addSharedDirectory(sdi);
        }
    }

    close();
}
