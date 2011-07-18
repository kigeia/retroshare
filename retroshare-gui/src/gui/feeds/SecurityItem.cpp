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

#include <QDateTime>
#include <QTimer>

#include "SecurityItem.h"
#include "FeedHolder.h"
#include "../RsAutoUpdatePage.h"
#include "gui/msgs/MessageComposer.h"
#include "gui/common/StatusDefs.h"

#include "gui/notifyqt.h"

#include <retroshare/rsmsgs.h>
#include <retroshare/rspeers.h>

#include <sstream>

/*****
 * #define DEBUG_ITEM 1
 ****/

/** Constructor */
SecurityItem::SecurityItem(FeedHolder *parent, uint32_t feedId, std::string gpgId, std::string sslId, uint32_t type, bool isHome)
:QWidget(NULL), mParent(parent), mFeedId(feedId),
	mSslId(sslId), mGpgId(gpgId), mType(type), mIsHome(isHome)
{
    /* Invoke the Qt Designer generated object setup routine */
    setupUi(this);
  
    messageframe->setVisible(false);
    sendmsgButton->setEnabled(false);

    /* general ones */
    connect( expandButton, SIGNAL( clicked( void ) ), this, SLOT( toggle ( void ) ) );
    connect( clearButton, SIGNAL( clicked( void ) ), this, SLOT( removeItem ( void ) ) );

    /* specific ones */
    connect( chatButton, SIGNAL( clicked( void ) ), this, SLOT( openChat ( void ) ) );
    connect( actionNew_Message, SIGNAL( triggered( ) ), this, SLOT( sendMsg ( void ) ) );

    connect( quickmsgButton, SIGNAL( clicked( ) ), this, SLOT( togglequickmessage() ) );
    connect( cancelButton, SIGNAL( clicked( ) ), this, SLOT( togglequickmessage() ) );

    connect( sendmsgButton, SIGNAL( clicked( ) ), this, SLOT( sendMessage() ) );

    connect(NotifyQt::getInstance(), SIGNAL(peerHasNewAvatar(const QString&)), this, SLOT(updateAvatar(const QString&)));
    
    QMenu *msgmenu = new QMenu();
    msgmenu->addAction(actionNew_Message);

    quickmsgButton->setMenu(msgmenu);

    small();
    updateItemStatic();
    updateItem();
    updateAvatar(QString::fromStdString(mGpgId));
}


bool SecurityItem::isSame(const std::string &sslId, uint32_t type)
{
	if ((mSslId == sslId) && (mType == type))
	{
		return true;
	}
	return false;
}


void SecurityItem::updateItemStatic()
{
	if (!rsPeers)
		return;


	/* fill in */
#ifdef DEBUG_ITEM
	std::cerr << "SecurityItem::updateItemStatic()";
	std::cerr << std::endl;
#endif
	QString title;

	switch(mType)
	{
		case SEC_TYPE_CONNECT_ATTEMPT:
			title = tr("Connect Attempt");
			break;
		case SEC_TYPE_AUTH_DENIED:
			title = tr("Not Yet Friends");
			break;
		case SEC_TYPE_UNKNOWN_IN:
			title = tr("Unknown (Incoming) Connect Attempt");
			break;
		case SEC_TYPE_UNKNOWN_OUT:
			title = tr("Unknown (Outgoing) Connect Attempt");
			break;
		default:
			title = tr("Unknown Security Issue");
			break;
	}

	titleLabel->setText(title);


	RsPeerDetails details;
	if (rsPeers->getPeerDetails(mGpgId, details))
	{
		
		/* set textcolor for peername  */
    		QString nameStr("<span style=\"font-size:14pt; font-weight:500;"
                               "color:#990033;\">%1</span>");
	
    		/* set Blog name */
    		QString peername =  QString::fromStdString(details.name);
    		peernameLabel->setText(nameStr.arg(peername));

		QDateTime date = QDateTime::fromTime_t(details.lastConnect);
		QString stime = date.toString(Qt::LocalDate);
		lastLabel-> setText(stime);

		/* expanded Info */
		nameLabel->setText(QString::fromUtf8(details.name.c_str()));
		idLabel->setText(QString::fromStdString(details.id));
		locLabel->setText(QString::fromUtf8(details.location.c_str()));
	}
	else
	{
		/* it is very likely that we will end up here for some of the
		 * Unknown peer cases.... so allow them here
	 	 */

		QDateTime date = QDateTime::currentDateTime();
		QString stime = date.toString(Qt::LocalDate);
		lastLabel-> setText(stime);
		nameLabel->setText(QString::fromStdString(mGpgId));
		idLabel->setText(QString::fromStdString(mSslId));

                statusLabel->setText(tr("Unknown Peer"));
                trustLabel->setText(tr("Unknown Peer"));
                locLabel->setText(tr("Unknown Peer"));
                ipLabel->setText(tr("Unknown Peer"));
                connLabel->setText(tr("Unknown Peer"));

		chatButton->setEnabled(false);

	}

	if (mIsHome)
	{
		/* disable buttons */
		clearButton->setEnabled(false);
		//gotoButton->setEnabled(false);

		/* disable buttons */
		clearButton->hide();
	}

}


void SecurityItem::updateItem()
{
	if (!rsPeers)
		return;

	/* fill in */
#ifdef DEBUG_ITEM
	std::cerr << "SecurityItem::updateItem()";
	std::cerr << std::endl;
#endif
	if(!RsAutoUpdatePage::eventsLocked()) {
		RsPeerDetails details;
		if (!rsPeers->getPeerDetails(mGpgId, details))
		{
			return;
		}

		/* top Level info */
		QString status = StatusDefs::peerStateString(details.state);

#if 0
		/* Append additional status info from status service */
		StatusInfo statusInfo;
		if ((rsStatus) && (rsStatus->getStatus(*it, statusInfo)))
		{
			status.append(QString::fromStdString("/" + RsStatusString(statusInfo.status)));
		}
#endif
		statusLabel->setText(status);
		trustLabel->setText(QString::fromStdString(
			RsPeerTrustString(details.trustLvl)));

		{
			std::ostringstream out;
			out << details.localAddr << ":";
			out << details.localPort << "/";
			out << details.extAddr << ":";
			out << details.extPort;
			ipLabel->setText(QString::fromStdString(out.str()));
		}

		connLabel->setText(StatusDefs::connectStateString(details));

		/* do buttons */
		chatButton->setEnabled(details.state & RS_PEER_STATE_CONNECTED);
		if (details.state & RS_PEER_STATE_FRIEND)
		{
			//addButton->setEnabled(false);
			//removeButton->setEnabled(true);
			quickmsgButton->setEnabled(true);
		}
		else
		{
			//addButton->setEnabled(true);
			//removeButton->setEnabled(false);
			quickmsgButton->setEnabled(false);
		}
	}

	/* slow Tick  */
	int msec_rate = 10129;
	
	QTimer::singleShot( msec_rate, this, SLOT(updateItem( void ) ));
	return;
}

void SecurityItem::small()
{
	expandFrame->hide();
}

void SecurityItem::toggle()
{
	if (expandFrame->isHidden())
	{
		expandFrame->show();
		expandButton->setIcon(QIcon(QString(":/images/edit_remove24.png")));
            expandButton->setToolTip(tr("Hide"));
	}
	else
	{
		expandFrame->hide();
		expandButton->setIcon(QIcon(QString(":/images/edit_add24.png")));
            expandButton->setToolTip(tr("Expand"));
	}
}


void SecurityItem::removeItem()
{
#ifdef DEBUG_ITEM
	std::cerr << "SecurityItem::removeItem()";
	std::cerr << std::endl;
#endif
	hide();
	if (mParent)
	{
		mParent->deleteFeedItem(this, mFeedId);
	}
}


void SecurityItem::gotoHome()
{
#ifdef DEBUG_ITEM
	std::cerr << "SecurityItem::gotoHome()";
	std::cerr << std::endl;
#endif
}

/*********** SPECIFIC FUNCTIOSN ***********************/

void SecurityItem::addFriend()
{
#ifdef DEBUG_ITEM
	std::cerr << "SecurityItem::addFriend()";
	std::cerr << std::endl;
#endif
}



void SecurityItem::removeFriend()
{
#ifdef DEBUG_ITEM
	std::cerr << "SecurityItem::removeFriend()";
	std::cerr << std::endl;
#endif
}



void SecurityItem::sendMsg()
{
#ifdef DEBUG_ITEM
	std::cerr << "SecurityItem::sendMsg()";
	std::cerr << std::endl;
#endif

	if (mParent)
	{
		//mParent->openMsg(FEEDHOLDER_MSG_MESSAGE, mPeerId, "");

    MessageComposer *nMsgDialog = MessageComposer::newMsg();
    if (nMsgDialog == NULL) {
        return;
    }

    nMsgDialog->addRecipient(MessageComposer::TO, mGpgId, false);
    nMsgDialog->show();
    nMsgDialog->activateWindow();

    /* window will destroy itself! */
	}
}


void SecurityItem::openChat()
{
#ifdef DEBUG_ITEM
	std::cerr << "SecurityItem::openChat()";
	std::cerr << std::endl;
#endif
	if (mParent)
	{
		mParent->openChat(mGpgId);
	}
}

void SecurityItem::updateAvatar(const QString &peer_id)
{
   if (peer_id.toStdString() != mGpgId) {
       /* it 's not me */
       return;
   }

   unsigned char *data = NULL;
   int size = 0 ;

   rsMsgs->getAvatarData(mGpgId,data,size); 


   if(size != 0)
   {   
    // set the image
    QPixmap pix ;
    pix.loadFromData(data,size,"PNG") ;
    avatar_label->setPixmap(pix);   
    delete[] data ;

   }
   else
   {
     avatar_label->setPixmap(QPixmap(":/images/user/personal64.png"));
   }


}  

void SecurityItem::togglequickmessage()
{
	if (messageframe->isHidden())
	{
        messageframe->setVisible(true);
    }
	else
	{
        messageframe->setVisible(false);
    }	

}

void SecurityItem::sendMessage()
{
    /* construct a message */
    MessageInfo mi;
    
    mi.title = tr("Quick Message").toStdWString();
    mi.msg =   quickmsgText->toHtml().toStdWString();
    mi.msgto.push_back(mGpgId);       
    
    rsMsgs->MessageSend(mi);

    quickmsgText->clear();
    messageframe->setVisible(false);
}

void SecurityItem::on_quickmsgText_textChanged()
{
    if (quickmsgText->toPlainText().isEmpty())
    {
        sendmsgButton->setEnabled(false);
    }
    else
    {
        sendmsgButton->setEnabled(true);
    }
}
