/****************************************************************
 *  RetroShare is distributed under the following license:
 *
 *  Copyright (C) 2006, crypton
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


#include "rshare.h"
#include "MessagesDialog.h"
#include "msgs/ChanMsgDialog.h"
#include "gui/toaster/MessageToaster.h"


#include "rsiface/rsiface.h"
#include <sstream>

#include <QContextMenuEvent>
#include <QMenu>
#include <QCursor>
#include <QPoint>
#include <QMouseEvent>
#include <QPixmap>
#include <QDateTime>

/* Images for context menu icons */
#define IMAGE_MESSAGE        ":/images/folder-draft.png"
#define IMAGE_MESSAGEREPLY   ":/images/mail_reply.png"
#define IMAGE_MESSAGEREMOVE  ":/images/mail_delete.png"
#define IMAGE_DOWNLOAD       ":/images/start.png"
#define IMAGE_DOWNLOADALL    ":/images/startall.png"


/** Constructor */
MessagesDialog::MessagesDialog(QWidget *parent)
: MainPage(parent)
{
  /* Invoke the Qt Designer generated object setup routine */
  ui.setupUi(this);

  connect( ui.msgWidget, SIGNAL( customContextMenuRequested( QPoint ) ), this, SLOT( messageslistWidgetCostumPopupMenu( QPoint ) ) );
  connect( ui.msgList, SIGNAL( customContextMenuRequested( QPoint ) ), this, SLOT( msgfilelistWidgetCostumPopupMenu( QPoint ) ) );
  connect( ui.msgWidget, SIGNAL( itemClicked ( QTreeWidgetItem *, int) ), this, SLOT( updateMessages ( QTreeWidgetItem *, int) ) );
  connect( ui.listWidget, SIGNAL( currentRowChanged ( int) ), this, SLOT( changeBox ( int) ) );
  
  
  connect(ui.newmessageButton, SIGNAL(clicked()), this, SLOT(newmessage()));
  connect(ui.removemessageButton, SIGNAL(clicked()), this, SLOT(removemessage()));
  

  mCurrCertId = "";
  mCurrMsgId  = "";
  
  /* hide the Tree +/- */
  ui.msgList -> setRootIsDecorated( false );


  /* Hide platform specific features */
#ifdef Q_WS_WIN

#endif
}

void MessagesDialog::messageslistWidgetCostumPopupMenu( QPoint point )
{

      QMenu contextMnu( this );
      QMouseEvent *mevent = new QMouseEvent( QEvent::MouseButtonPress, point, Qt::RightButton, Qt::RightButton, Qt::NoModifier );

      newmsgAct = new QAction(QIcon(IMAGE_MESSAGE), tr( "New Message" ), this );
      connect( newmsgAct , SIGNAL( triggered() ), this, SLOT( newmessage() ) );
      
      replytomsgAct = new QAction(QIcon(IMAGE_MESSAGEREPLY), tr( "Reply to Message" ), this );
      connect( replytomsgAct , SIGNAL( triggered() ), this, SLOT( replytomessage() ) );
      
      removemsgAct = new QAction(QIcon(IMAGE_MESSAGEREMOVE), tr( "Remove Message" ), this );
      connect( removemsgAct , SIGNAL( triggered() ), this, SLOT( removemessage() ) );
      


      contextMnu.clear();
      contextMnu.addAction( newmsgAct);
      contextMnu.addAction( replytomsgAct);
      contextMnu.addAction( removemsgAct);
      contextMnu.exec( mevent->globalPos() );
}


void MessagesDialog::msgfilelistWidgetCostumPopupMenu( QPoint point )
{

      QMenu contextMnu( this );
      QMouseEvent *mevent = new QMouseEvent( QEvent::MouseButtonPress, point, Qt::RightButton, Qt::RightButton, Qt::NoModifier );

//      getRecAct = new QAction(QIcon(IMAGE_DOWNLOAD), tr( "Download" ), this );
//      connect( getRecAct , SIGNAL( triggered() ), this, SLOT( getcurrentrecommended() ) );
      
      getAllRecAct = new QAction(QIcon(IMAGE_DOWNLOADALL), tr( "Download All" ), this );
      connect( getAllRecAct , SIGNAL( triggered() ), this, SLOT( getallrecommended() ) );


      contextMnu.clear();
//      contextMnu.addAction( getRecAct);
      contextMnu.addAction( getAllRecAct);
      contextMnu.exec( mevent->globalPos() );
}

void MessagesDialog::newmessage()
{
    ChanMsgDialog *nMsgDialog = new ChanMsgDialog(true);

    /* fill it in */
    //std::cerr << "MessagesDialog::newmessage()" << std::endl;
    nMsgDialog->newMsg();
    nMsgDialog->show();

    /* window will destroy itself! */
}

void MessagesDialog::replytomessage()
{
	/* put msg on msgBoard, and switch to it. */


}


/* download the recommendations... */
void MessagesDialog::getcurrentrecommended()
{
   

}

void MessagesDialog::getallrecommended()
{
	/* get Message */
	rsiface->lockData();   /* Lock Interface */

	const MessageInfo *mi = 
		rsiface->getMessage(mCurrCertId, mCurrMsgId);
	if (!mi)
	{
		rsiface->unlockData();   /* Unlock Interface */
		return;
	}

        const std::list<FileInfo> &recList = mi->files;
	std::list<FileInfo>::const_iterator it;

	std::list<std::string> fnames;
	std::list<std::string> hashes;
	std::list<int>         sizes;

	for(it = recList.begin(); it != recList.end(); it++)
	{
		fnames.push_back(it->fname);
		hashes.push_back(it->hash);
		sizes.push_back(it->size);
	}

	rsiface->unlockData(); /* Unlock Interface */

	/* now do requests */
	std::list<std::string>::const_iterator fit;
	std::list<std::string>::const_iterator hit;
	std::list<int>::const_iterator sit;

	for(fit = fnames.begin(), hit = hashes.begin(), sit = sizes.begin(); 
		fit != fnames.end(); fit++, hit++, sit++)
	{
        	rsicontrol -> FileRequest(*fit, *hit, *sit, "");
	}
}

void MessagesDialog::changeBox( int newrow )
{
	//std::cerr << "MessagesDialog::changeBox()" << std::endl;
	insertMessages();
	insertMsgTxtAndFiles();
}

void MessagesDialog::insertMessages()
{
	rsiface->lockData(); /* Lock Interface */

	std::list<MessageInfo>::const_iterator it;
	const std::list<MessageInfo> &msgs = rsiface->getMessages();

	/* get a link to the table */
        QTreeWidget *msgWidget = ui.msgWidget;

	/* get the MsgId of the current one ... */


	std::string cid;
	std::string mid;

	bool oldSelected = getCurrentMsg(cid, mid);
	QTreeWidgetItem *newSelected = NULL;

	/* remove old items ??? */

	int listrow = ui.listWidget -> currentRow();

	//std::cerr << "MessagesDialog::insertMessages()" << std::endl;
	//std::cerr << "Current Row: " << listrow << std::endl;

	/* check the mode we are in */
	unsigned int msgbox = 0;
	switch(listrow)
	{
		case 3:
			msgbox = RS_MSG_SENTBOX;
			break;
		case 2:
			msgbox = RS_MSG_DRAFTBOX;
			break;
		case 1:
			msgbox = RS_MSG_OUTBOX;
			break;
		case 0:
		default:
			msgbox = RS_MSG_INBOX;
			break;
	}

        QList<QTreeWidgetItem *> items;
	for(it = msgs.begin(); it != msgs.end(); it++)
	{
		/* check the message flags, to decide which
		 * group it should go in...
		 *
		 * InBox 
		 * OutBox 
		 * Drafts 
		 * Sent 
		 *
		 * FLAGS = OUTGOING.
		 * 	-> Outbox/Drafts/Sent
		 * 	  + SENT -> Sent
		 *	  + IN_PROGRESS -> Draft.
		 *	  + nuffing -> Outbox.
		 * FLAGS = INCOMING = (!OUTGOING)
		 * 	-> + NEW -> Bold.
		 *
		 */

		if ((it -> msgflags & RS_MSG_BOXMASK) != msgbox)
		{
			//std::cerr << "Msg from other box: " << it->msgflags;
			//std::cerr << std::endl;
			continue;
		}
		/* make a widget per friend */
           	QTreeWidgetItem *item = new QTreeWidgetItem((QTreeWidget*)0);

		/* So Text should be:
		 * (1) Msg / Broadcast
		 * (1b) Person / Channel Name
		 * (2) Rank
		 * (3) Date
		 * (4) Title
		 * (5) Msg
		 * (6) File Count
		 * (7) File Total
		 */

		// Date First.... (for sorting)
		{
			QDateTime qtime;
			qtime.setTime_t(it->ts);
			QString timestamp = qtime.toString("yyyy-MM-dd hh:mm:ss");
			item -> setText(0, timestamp);
		}

		//  From ....
		{
			std::ostringstream out;
			out << it -> srcname;
			item -> setText(1, QString::fromStdString(out.str()));
		}

		item -> setText(2, QString::fromStdString(it->title));

		// No of Files.
		{
			std::ostringstream out;
			out << it -> count;
			item -> setText(3, QString::fromStdString(out.str()));
		}

		// Size.
		// Msg.
		// Rank
		{
			std::ostringstream out;
			out << it -> size;
			item -> setText(4, QString::fromStdString(out.str()));
		}

		/* strip out the \n and \r symbols */
		std::string tmsg = it -> msg;
		for(int i = 0; i < tmsg.length(); i++)
		{
			if ((tmsg[i] == '\n') ||
			    (tmsg[i] == '\r'))
			{
			   tmsg[i] = ' ';
			}
		}
		item -> setText(5, QString::fromStdString(tmsg));

		{
			std::ostringstream out;
			out << "5"; // RANK 
			item -> setText(6, QString::fromStdString(out.str()));
		}

		{
			std::ostringstream out;
			out << it -> id;
			item -> setText(7, QString::fromStdString(out.str()));
		}

		{
			std::ostringstream out;
			out << it -> msgId;
			item -> setText(8, QString::fromStdString(out.str()));
			if ((oldSelected) && (mid == out.str()))
			{
				newSelected = item;
			}
		}

		if (it -> msgflags & RS_MSG_NEW)
		{
			for(int i = 0; i < 10; i++)
			{
				QFont qf = item->font(i);
				qf.setBold(true);
				item->setFont(i, qf);

				//std::cerr << "Setting Item BOLD!" << std::endl;
			}
		}

		/* add to the list */
		items.append(item);
	}

	/* add the items in! */
	msgWidget->clear();
	msgWidget->insertTopLevelItems(0, items);

	if (newSelected)
	{
		msgWidget->setCurrentItem(newSelected);
	}

	rsiface->unlockData(); /* UnLock Interface */
}

void MessagesDialog::updateMessages( QTreeWidgetItem * item, int column )
{
	//std::cerr << "MessagesDialog::insertMsgTxtAndFiles()" << std::endl;
	insertMsgTxtAndFiles();
}


void MessagesDialog::insertMsgTxtAndFiles()
{
	/* Locate the current Message */
	QTreeWidget *msglist = ui.msgWidget;

	//std::cerr << "MessagesDialog::insertMsgTxtAndFiles()" << std::endl;


	/* get its Ids */
	std::string cid;
	std::string mid;

	QTreeWidgetItem *qtwi = msglist -> currentItem();
	if (!qtwi)
	{
		/* blank it */
		ui.msgText->setText("");
		ui.msgList->clear();
		return;
	}
	else
	{
		cid = qtwi -> text(7).toStdString();
		mid = qtwi -> text(8).toStdString();
	}

	/* Save the Data.... for later */

	mCurrCertId = cid;
	mCurrMsgId  = mid;

	/* get Message */
	rsiface->lockData();   /* Lock Interface */

	const MessageInfo *mi = NULL;
	mi = rsiface->getMessage(cid, mid);
	if (!mi)
	{
		rsiface->unlockData();   /* Unlock Interface */
		return;
	}

        const std::list<FileInfo> &recList = mi->files;
	std::list<FileInfo>::const_iterator it;

	/* get a link to the table */
	QTreeWidget *tree = ui.msgList;

	/* get the MessageInfo */

        tree->clear(); 
        tree->setColumnCount(5); 

        QList<QTreeWidgetItem *> items;
	for(it = recList.begin(); it != recList.end(); it++)
	{
		/* make a widget per person */
           	QTreeWidgetItem *item = new QTreeWidgetItem((QTreeWidget*)0);
		/* (0) Filename */
		item -> setText(0, QString::fromStdString(it->fname));
		//std::cerr << "Msg FileItem(" << it->fname.length() << ") :" << it->fname << std::endl;
			
		/* (1) Size */
		{
			std::ostringstream out;
			out << it->size;
			item -> setText(1, QString::fromStdString(out.str()));
		}
		/* (2) Rank */
		{
			std::ostringstream out;
			out << it->rank;
			item -> setText(2, QString::fromStdString(out.str()));
		}
			
		item -> setText(3, QString::fromStdString(it->hash));
			
		/* add to the list */
		items.append(item);
	}

	/* add the items in! */
	tree->insertTopLevelItems(0, items);


	/* add the Msg */
	std::string msgtext = "Title:  " + mi -> title;
	msgtext +=            "\nHdr:   " + mi -> header;
	msgtext += "\n-----------------\n" + mi -> msg;

	ui.msgText->setText(QString::fromStdString(msgtext));

	rsiface->unlockData();   /* Unlock Interface */


	/* finally mark message as read! */
	rsicontrol -> MessageRead(mid);
}


bool MessagesDialog::getCurrentMsg(std::string &cid, std::string &mid)
{
	/* Locate the current Message */
	QTreeWidget *msglist = ui.msgWidget;

	//std::cerr << "MessagesDialog::getCurrentMsg()" << std::endl;

	/* get its Ids */
	QTreeWidgetItem *qtwi = msglist -> currentItem();
	if (qtwi)
	{
		cid = qtwi -> text(7).toStdString();
		mid = qtwi -> text(8).toStdString();
		return true;
	}
	return false;
}


void MessagesDialog::removemessage()
{
	//std::cerr << "MessagesDialog::removemessage()" << std::endl;
	std::string cid, mid;
	if (!getCurrentMsg(cid, mid))
	{
		//std::cerr << "MessagesDialog::removemessage()";
		//std::cerr << " No Message selected" << std::endl;
		return;
	}

	rsicontrol -> MessageDelete(mid);

	return;
}


void MessagesDialog::markMsgAsRead()
{
	//std::cerr << "MessagesDialog::markMsgAsRead()" << std::endl;
	std::string cid, mid;
	if (!getCurrentMsg(cid, mid))
	{
		//std::cerr << "MessagesDialog::markMsgAsRead()";
		//std::cerr << " No Message selected" << std::endl;
		return;
	}

	rsicontrol -> MessageRead(mid);

	return;
}


