/****************************************************************
 *  RetroShare is distributed under the following license:
 *
 *  Copyright (C) 2008 Robert Fernie
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

#include "CreateForumMsg.h"

#include <QMenu>
#include <QMessageBox>
#include <QFile>
#include <QFileDialog>
#include <QDesktopWidget>
#include <QDropEvent>

#include <retroshare/rsforums.h>
#include <retroshare/rsfiles.h>

#include "gui/settings/rsharesettings.h"
#include "gui/RetroShareLink.h"
#include "gui/feeds/AttachFileItem.h"
#include "gui/common/Emoticons.h"

#include "util/misc.h"

#include <sys/stat.h>


/** Constructor */
CreateForumMsg::CreateForumMsg(std::string fId, std::string pId)
: QMainWindow(NULL), mForumId(fId), mParentId(pId)
{
    /* Invoke the Qt Designer generated object setup routine */
    ui.setupUi(this);
    setAttribute(Qt::WA_DeleteOnClose, true);

    Settings->loadWidgetInformation(this);

    connect( ui.forumMessage, SIGNAL( customContextMenuRequested( QPoint ) ), this, SLOT( forumMessageCostumPopupMenu( QPoint ) ) );

    // connect up the buttons.
    connect( ui.postmessage_action, SIGNAL( triggered (bool) ), this, SLOT( createMsg( ) ) );
    connect( ui.close_action, SIGNAL( triggered (bool) ), this, SLOT( cancelMsg( ) ) );
    connect( ui.emoticonButton, SIGNAL(clicked()), this, SLOT(smileyWidgetForums()));
    connect( ui.attachFileButton, SIGNAL(clicked() ), this , SLOT(addFile()));
    connect( ui.pastersButton, SIGNAL(clicked() ), this , SLOT(pasteLink()));

    newMsg();
}

/** context menu searchTablewidget2 **/
void CreateForumMsg::forumMessageCostumPopupMenu( QPoint point )
{
    QMenu contextMnu (this);

    QAction *pasteLinkAct = new QAction(QIcon(":/images/pasterslink.png"), tr( "Paste retroshare Link" ), &contextMnu );
    QAction *pasteLinkFullAct = new QAction(QIcon(":/images/pasterslink.png"), tr( "Paste retroshare Link Full" ), &contextMnu );

    if (RSLinkClipboard::empty()) {
        pasteLinkAct->setDisabled (true);
        pasteLinkFullAct->setDisabled (true);
    } else {
        connect(pasteLinkAct , SIGNAL(triggered()), this, SLOT(pasteLink()));
        connect(pasteLinkFullAct , SIGNAL(triggered()), this, SLOT(pasteLinkFull()));
    }

    contextMnu.addAction(pasteLinkAct);
    contextMnu.addAction(pasteLinkFullAct);

    contextMnu.exec(QCursor::pos());
}


void  CreateForumMsg::newMsg()
{
    /* clear all */
    ForumInfo fi;
    if (rsForums->getForumInfo(mForumId, fi))
    {
        ForumMsgInfo msg;

        QString name = QString::fromStdWString(fi.forumName);
        QString subj;
        if ((mParentId != "") && (rsForums->getForumMessage(mForumId, mParentId, msg)))
        {
            QString title = QString::fromStdWString(msg.title);
            name += " " + tr("In Reply to") + ": ";
            name += title;

            QString text = title;

            if (text.startsWith("Re:", Qt::CaseInsensitive))
            {
                subj = title;
            }
            else
            {
                subj = "Re: " + title;
            }

        }

        ui.forumName->setText(misc::removeNewLine(name));
        ui.forumSubject->setText(misc::removeNewLine(subj));

        if (!ui.forumSubject->text().isEmpty())
        {
            ui.forumMessage->setFocus();
        }
        else
        {
            ui.forumSubject->setFocus();
        }

        if (fi.forumFlags & RS_DISTRIB_AUTHEN_REQ)
        {
            ui.signBox->setChecked(true);
            ui.signBox->setEnabled(false);
        }
        else
        {
            ui.signBox->setEnabled(true);
        }
    }

    ui.forumMessage->setText("");
}

void  CreateForumMsg::createMsg()
{
    QString name = misc::removeNewLine(ui.forumSubject->text());
    QString desc = ui.forumMessage->toHtml();

    if(name.isEmpty())
    {	/* error message */
        QMessageBox::warning(this, tr("RetroShare"),tr("Please set a Forum Subject and Forum Message"),
                             QMessageBox::Ok, QMessageBox::Ok);

        return; //Don't add  a empty Subject!!
    }

    ForumMsgInfo msgInfo;

    msgInfo.forumId = mForumId;
    msgInfo.threadId = "";
    msgInfo.parentId = mParentId;
    msgInfo.msgId = "";

    msgInfo.title = name.toStdWString();
    msgInfo.msg = desc.toStdWString();
    msgInfo.msgflags = 0;

    if (ui.signBox->isChecked())
    {
        msgInfo.msgflags = RS_DISTRIB_AUTHEN_REQ;
    }

    if ((msgInfo.msg == L"") && (msgInfo.title == L""))
        return; /* do nothing */

    if (rsForums->ForumMessageSend(msgInfo) == true) {
        close();
    }
}

void CreateForumMsg::closeEvent (QCloseEvent * event)
{
    Settings->saveWidgetInformation(this);
}

void CreateForumMsg::cancelMsg()
{
    close();
}

void CreateForumMsg::smileyWidgetForums()
{
    Emoticons::showSmileyWidget(this, ui.emoticonButton, SLOT(addSmileys()), false);
}

void CreateForumMsg::addSmileys()
{
    ui.forumMessage->textCursor().insertText(qobject_cast<QPushButton*>(sender())->toolTip().split("|").first());
}

void CreateForumMsg::addFile()
{
	// select a file
	QString qfile = QFileDialog::getOpenFileName(this, tr("Add Extra File"), "", "", 0,
				QFileDialog::DontResolveSymlinks);
	std::string filePath = qfile.toStdString();
	if (filePath != "")
	{
	    CreateForumMsg::addAttachment(filePath);
	}
}

void CreateForumMsg::addAttachment(std::string filePath) {
	    /* add a AttachFileItem to the attachment section */
	    std::cerr << "CreateForumMsg::addFile() hashing file.";
	    std::cerr << std::endl;

	    /* add widget in for new destination */
	    AttachFileItem *file = new AttachFileItem(filePath);
	    //file->

	    ui.verticalLayout->addWidget(file, 1, 0);

	    //when the file is local or is finished hashing, call the fileHashingFinished method to send a forum message
	    if (file->getState() == AFI_STATE_LOCAL) {
		fileHashingFinished(file);
	    } else {
		QObject::connect(file,SIGNAL(fileFinished(AttachFileItem *)),this, SLOT(fileHashingFinished(AttachFileItem *))) ;
	    }
}

void CreateForumMsg::fileHashingFinished(AttachFileItem* file) {
	std::cerr << "CreateForumMsg::fileHashingFinished() started.";
	std::cerr << std::endl;

	//check that the file is ok tos end
	if (file->getState() == AFI_STATE_ERROR) {
	#ifdef CHAT_DEBUG
		    std::cerr << "CreateForumMsg::fileHashingFinished error file is not hashed.";
	#endif
	    return;
	}

	//convert fileSize from uint_64 to string for html link
	char fileSizeChar [100];
	sprintf(fileSizeChar, "%llu", (unsigned long long int)file->FileSize());
	std::string fileSize = *(&fileSizeChar);

	std::string mesgString = "<a href='retroshare://file|" + (file->FileName()) + "|" + fileSize + "|" + (file->FileHash()) + "'>"
	+ "retroshare://file|" + (file->FileName()) + "|" + fileSize +  "|" + (file->FileHash())  + "</a>" + "<br>";
#ifdef CHAT_DEBUG
	    std::cerr << "CreateForumMsg::anchorClicked mesgString : " << mesgString << std::endl;
#endif

    ui.forumMessage->textCursor().insertHtml(QString::fromStdString(mesgString));

	ui.forumMessage->setFocus( Qt::OtherFocusReason );

}

void CreateForumMsg::dropEvent(QDropEvent *event)
{
	if (!(Qt::CopyAction & event->possibleActions()))
	{
                std::cerr << "CreateForumMsg::dropEvent() Rejecting uncopyable DropAction" << std::endl;

		/* can't do it */
		return;
	}

        std::cerr << "CreateForumMsg::dropEvent() Formats" << std::endl;
	QStringList formats = event->mimeData()->formats();
	QStringList::iterator it;
	for(it = formats.begin(); it != formats.end(); it++)
	{
                std::cerr << "Format: " << (*it).toStdString() << std::endl;
	}

	if (event->mimeData()->hasUrls())
	{
                std::cerr << "CreateForumMsg::dropEvent() Urls:" << std::endl;

		QList<QUrl> urls = event->mimeData()->urls();
		QList<QUrl>::iterator uit;
		for(uit = urls.begin(); uit != urls.end(); uit++)
		{
			std::string localpath = uit->toLocalFile().toStdString();
                        std::cerr << "Whole URL: " << uit->toString().toStdString() << std::endl;
                        std::cerr << "or As Local File: " << localpath << std::endl;

			if (localpath.size() > 0)
			{
				struct stat buf;
				//Check that the file does exist and is not a directory
				if ((-1 == stat(localpath.c_str(), &buf))) {
				    std::cerr << "CreateForumMsg::dropEvent() file does not exists."<< std::endl;
				    QMessageBox mb(tr("Drop file error."), tr("File not found or file name not accepted."),QMessageBox::Information,QMessageBox::Ok,0,0,this);
				    mb.exec();
				} else if (S_ISDIR(buf.st_mode)) {
				    std::cerr << "CreateForumMsg::dropEvent() directory not accepted."<< std::endl;
				    QMessageBox mb(tr("Drop file error."), tr("Directory can't be dropped, only files are accepted."),QMessageBox::Information,QMessageBox::Ok,0,0,this);
				    mb.exec();
				} else {
				    CreateForumMsg::addAttachment(localpath);
				}
			}
		}
	}

	event->setDropAction(Qt::CopyAction);
	event->accept();
}

void CreateForumMsg::dragEnterEvent(QDragEnterEvent *event)
{
	/* print out mimeType */
        std::cerr << "CreateForumMsg::dragEnterEvent() Formats" << std::endl;
	QStringList formats = event->mimeData()->formats();
	QStringList::iterator it;
	for(it = formats.begin(); it != formats.end(); it++)
	{
                std::cerr << "Format: " << (*it).toStdString() << std::endl;
	}

	if (event->mimeData()->hasUrls())
	{
                std::cerr << "CreateForumMsg::dragEnterEvent() Accepting Urls" << std::endl;
		event->acceptProposedAction();
	}
	else
	{
                std::cerr << "CreateForumMsg::dragEnterEvent() No Urls" << std::endl;
	}
}

void CreateForumMsg::pasteLink()
{
	ui.forumMessage->insertHtml(RSLinkClipboard::toHtml()) ;
}

void CreateForumMsg::pasteLinkFull()
{
	ui.forumMessage->insertHtml(RSLinkClipboard::toHtmlFull()) ;
}

