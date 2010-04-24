/****************************************************************
 *  RetroShare is distributed under the following license:
 *
 *  Copyright (C) 2006 - 2009 RetroShare Team
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

#include "DirectoriesPage.h"
#include "gui/ShareManager.h"

#include "rshare.h"
#include "rsiface/rsfiles.h"

#include <algorithm>

DirectoriesPage::DirectoriesPage(QWidget * parent, Qt::WFlags flags)
    : ConfigPage(parent, flags)
{
    ui.setupUi(this);
    setAttribute(Qt::WA_QuitOnClose, false);

    //load();

#ifdef TO_REMOVE
	ui.addButton->setToolTip(tr("Add a Share Directory"));
	ui.removeButton->setToolTip(tr("Remove Shared Directory"));
#endif

    ui.incomingButton->setToolTip(tr("Browse"));
    ui.partialButton->setToolTip(tr("Browse"));

    if (rsFiles->getShareDownloadDirectory())
    {
        ui.checkBox->setChecked(true);		/* signal not emitted */
    }
    else
    {
        ui.checkBox->setChecked(false);	/* signal not emitted */
    }

    connect(ui.incomingButton, SIGNAL(clicked( bool ) ), this , SLOT( setIncomingDirectory() ) );
    connect(ui.partialButton, SIGNAL(clicked( bool ) ), this , SLOT( setPartialsDirectory() ) );
    connect(ui.checkBox, SIGNAL(stateChanged(int)), this, SLOT(shareDownloadDirectory(int)));
    connect(ui.editButton, SIGNAL(clicked()), this, SLOT(editDirectories()));

  /* Hide platform specific features */
#ifdef Q_WS_WIN

#endif
}

void
DirectoriesPage::closeEvent (QCloseEvent * event)
{
    QWidget::closeEvent(event);
}

void DirectoriesPage::editDirectories()
{
	ShareManager::showYourself() ;
}


/** Saves the changes on this page */
bool DirectoriesPage::save(QString &errmsg)
{
	/* this is usefull especially when shared incoming files is
	 * default option and when the user don't check/uncheck the
	 * checkBox, so no signal is emitted to update the shared list */
	if (ui.checkBox->isChecked())
	{
		std::list<SharedDirInfo>::const_iterator it;
		std::list<SharedDirInfo> dirs;
		rsFiles->getSharedDirectories(dirs);

		bool found = false ;
		for(std::list<SharedDirInfo>::const_iterator it(dirs.begin());it!=dirs.end();++it)
			if((*it).filename == rsFiles->getDownloadDirectory())
			{
				found=true ;
				break ;
			}
		if(!found)
			rsFiles->shareDownloadDirectory();

		rsFiles->setShareDownloadDirectory(true);
	}
	else
	{
		rsFiles->unshareDownloadDirectory();
		rsFiles->setShareDownloadDirectory(false);
	}

	return true;
}

/** Loads the settings for this page */
void DirectoriesPage::load()
{
	std::list<SharedDirInfo>::const_iterator it;
	std::list<SharedDirInfo> dirs;
	rsFiles->getSharedDirectories(dirs);

	/* get a link to the table */
	QListWidget *listWidget = ui.dirList;

	/* save current index */
	QModelIndex rootIndex = listWidget->rootIndex();

	/* remove old items ??? */
	listWidget->clear();

	for(it = dirs.begin(); it != dirs.end(); it++)
	{
		/* (0) Dir Name */
		listWidget->addItem(QString::fromStdString((*it).filename));
	}

	/* set saved index */
	listWidget->setCurrentIndex(rootIndex);

	ui.incomingDir->setText(QString::fromStdString(rsFiles->getDownloadDirectory()));
	ui.partialsDir->setText(QString::fromStdString(rsFiles->getPartialsDirectory()));

	listWidget->update(); /* update display */
}

#ifdef TO_REMOVE
void DirectoriesPage::addShareDirectory()
{

	/* select a dir
	 */

	int ind;
 	QString qdir = QFileDialog::getExistingDirectory(this, tr("Add Shared Directory"), "",
 	                            QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
        ind=qdir.lastIndexOf("/");

	/* add it to the server */
	std::string dir = qdir.toStdString();
	if (dir != "")
	{
		rsFiles->addSharedDirectory(dir);
		load();
	}
}

void DirectoriesPage::removeShareDirectory()
{
	/* id current dir */
	/* ask for removal */
	QListWidget *listWidget = ui.dirList;
	QListWidgetItem *qdir = listWidget -> currentItem();
	if (qdir)
	{
		rsFiles->removeSharedDirectory( qdir->text().toStdString());
		load();
	}
}
#endif

void DirectoriesPage::setIncomingDirectory()
{
 	QString qdir = QFileDialog::getExistingDirectory(this, tr("Set Incoming Directory"), "",
				QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

	std::string dir = qdir.toStdString();
	if (dir != "")
	{
		rsFiles->setDownloadDirectory(dir);
		if (ui.checkBox->isChecked())
		{
			std::list<SharedDirInfo>::const_iterator it;
			std::list<SharedDirInfo> dirs;
			rsFiles->getSharedDirectories(dirs);

			bool found = false ;
			for(std::list<SharedDirInfo>::const_iterator it(dirs.begin());it!=dirs.end();++it)
				if((*it).filename == rsFiles->getDownloadDirectory())
				{
					found=true ;
					break ;
				}
			if(!found)
				rsFiles->shareDownloadDirectory();
		}
	}
	load();
}

void DirectoriesPage::setPartialsDirectory()
{
 	QString qdir = QFileDialog::getExistingDirectory(this, tr("Set Partials Directory"), "",
				QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

	std::string dir = qdir.toStdString();
	if (dir != "")
	{
		rsFiles->setPartialsDirectory(dir);
	}
	load();
}

void DirectoriesPage::shareDownloadDirectory(int state)
{
	if (state == Qt::Checked)
	{
		std::list<SharedDirInfo>::const_iterator it;
		std::list<SharedDirInfo> dirs;
		rsFiles->getSharedDirectories(dirs);

		bool found = false ;
		for(std::list<SharedDirInfo>::const_iterator it(dirs.begin());it!=dirs.end();++it)
			if((*it).filename == rsFiles->getDownloadDirectory())
			{
				found=true ;
				break ;
			}
		if(!found)
			rsFiles->shareDownloadDirectory();

		rsFiles->setShareDownloadDirectory(true);
	}
	else
	{
		rsFiles->unshareDownloadDirectory();
		rsFiles->setShareDownloadDirectory(false);
	}
	load();
}

