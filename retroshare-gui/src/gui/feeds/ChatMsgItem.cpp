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

#include "ChatMsgItem.h"
#include "FeedHolder.h"
#include "../RsAutoUpdatePage.h"
#include "gui/msgs/MessageComposer.h"

#include "gui/notifyqt.h"

#include <retroshare/rsmsgs.h>
#include <retroshare/rspeers.h>

#include <sstream>

/*****
 * #define DEBUG_ITEM 1
 ****/

/** Constructor */
ChatMsgItem::ChatMsgItem(FeedHolder *parent, uint32_t feedId, std::string peerId, std::string message, bool isHome)
:QWidget(NULL), mParent(parent), mFeedId(feedId),
	mPeerId(peerId), mIsHome(isHome)
{
    /* Invoke the Qt Designer generated object setup routine */
    setupUi(this);

    /* general ones */
    connect( clearButton, SIGNAL( clicked( void ) ), this, SLOT( removeItem ( void ) ) );

    /* specific ones */
    connect( chatButton, SIGNAL( clicked( void ) ), this, SLOT( openChat ( void ) ) );
    connect( msgButton, SIGNAL( clicked( void ) ), this, SLOT( sendMsg ( void ) ) );

    connect(NotifyQt::getInstance(), SIGNAL(peerHasNewAvatar(const QString&)), this, SLOT(updateAvatar(const QString&)));

    updateItemStatic();
    updateItem();
    updateAvatar(QString::fromStdString(mPeerId));
    insertChat(message);
}

void ChatMsgItem::updateItemStatic()
{
    if (!rsPeers)
        return;

    RsPeerDetails details;
    if (rsPeers->getPeerDetails(mPeerId, details))
    {

        /* set textcolor for peername  */
        QString nameStr("<span style=\"font-size:14pt; font-weight:500;"
                        "color:#990033;\">%1</span>");
	
        /* set Peer name */
        QString peername =  QString::fromStdString(details.name);
        peernameLabel->setText(nameStr.arg(peername));
    }
    else
    {
        chatButton->setEnabled(false);
        msgButton->setEnabled(false);
    }

    /* fill in */
#ifdef DEBUG_ITEM
    std::cerr << "ChatMsgItem::updateItemStatic()";
    std::cerr << std::endl;
#endif
}

void ChatMsgItem::updateItem()
{
    if (!rsPeers)
        return;

    /* fill in */
#ifdef DEBUG_ITEM
    std::cerr << "ChatMsgItem::updateItem()";
    std::cerr << std::endl;
#endif

    if(!RsAutoUpdatePage::eventsLocked()) {
        RsPeerDetails details;
        if (!rsPeers->getPeerDetails(mPeerId, details))
        {
            return;
        }

        /* do buttons */
        chatButton->setEnabled(details.state & RS_PEER_STATE_CONNECTED);
        if (details.state & RS_PEER_STATE_FRIEND)
        {
            msgButton->setEnabled(true);
        }
        else
        {
            msgButton->setEnabled(false);
        }
    }

    /* slow Tick  */
    int msec_rate = 10129;

    QTimer::singleShot( msec_rate, this, SLOT(updateItem( void ) ));
    return;
}

void ChatMsgItem::insertChat(std::string &message)
{
#ifdef DEBUG_ITEM
    std::cerr << "ChatMsgItem::insertChat(): " << msg << std::endl;
#endif

    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    timestampLabel->setText(timestamp);

    chatTextlabel->setText(QString::fromStdString(message));
}

void ChatMsgItem::removeItem()
{
#ifdef DEBUG_ITEM
	std::cerr << "ChatMsgItem::removeItem()";
	std::cerr << std::endl;
#endif
	hide();
	if (mParent)
	{
		mParent->deleteFeedItem(this, mFeedId);
	}
}

void ChatMsgItem::gotoHome()
{
#ifdef DEBUG_ITEM
	std::cerr << "ChatMsgItem::gotoHome()";
	std::cerr << std::endl;
#endif
}

/*********** SPECIFIC FUNCTIOSN ***********************/


void ChatMsgItem::sendMsg()
{
#ifdef DEBUG_ITEM
	std::cerr << "ChatMsgItem::sendMsg()";
	std::cerr << std::endl;
#endif

	if (mParent)
	{

    MessageComposer *nMsgDialog = new MessageComposer();
    nMsgDialog->newMsg();

    nMsgDialog->addRecipient( mPeerId ) ;
    nMsgDialog->show();
    nMsgDialog->activateWindow();

    /* window will destroy itself! */
	}
}


void ChatMsgItem::openChat()
{
#ifdef DEBUG_ITEM
	std::cerr << "ChatMsgItem::openChat()";
	std::cerr << std::endl;
#endif
	if (mParent)
	{
		mParent->openChat(mPeerId);
	}
}

void ChatMsgItem::updateAvatar(const QString &peer_id)
{
    if (peer_id.toStdString() != mPeerId) {
        /* it 's not me */
        return;
    }

    unsigned char *data = NULL;
    int size = 0 ;

    rsMsgs->getAvatarData(mPeerId,data,size);

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