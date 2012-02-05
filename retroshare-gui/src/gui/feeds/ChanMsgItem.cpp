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

#include "ChanMsgItem.h"

#include "FeedHolder.h"
#include "SubFileItem.h"
#include "gui/notifyqt.h"
#include "util/misc.h"
#include "gui/RetroShareLink.h"

#include <retroshare/rschannels.h>

#include <sstream>


/****
 * #define DEBUG_ITEM 1
 ****/

/** Constructor */
ChanMsgItem::ChanMsgItem(FeedHolder *parent, uint32_t feedId, const std::string &chanId, const std::string &msgId, bool isHome)
:QWidget(NULL), mParent(parent), mFeedId(feedId),
	mChanId(chanId), mMsgId(msgId), mIsHome(isHome)
{
	/* Invoke the Qt Designer generated object setup routine */
	setupUi(this);

	setAttribute ( Qt::WA_DeleteOnClose, true );

	m_inUpdateItemStatic = false;

	/* general ones */
	connect( expandButton, SIGNAL( clicked( void ) ), this, SLOT( toggle ( void ) ) );
	connect( clearButton, SIGNAL( clicked( void ) ), this, SLOT( removeItem ( void ) ) );

	/* specific */
	connect( unsubscribeButton, SIGNAL( clicked( void ) ), this, SLOT( unsubscribeChannel ( void ) ) );
	connect( downloadButton, SIGNAL( clicked( void ) ), this, SLOT( download ( void ) ) );
	connect( playButton, SIGNAL( clicked( void ) ), this, SLOT( play ( void ) ) );
	connect( copyLinkButton, SIGNAL( clicked( void ) ), this, SLOT( copyLink ( void ) ) );

	connect( readButton, SIGNAL( toggled(bool) ), this, SLOT( readToggled(bool) ) );
	connect( NotifyQt::getInstance(), SIGNAL(channelMsgReadSatusChanged(QString,QString,int)), this, SLOT(channelMsgReadSatusChanged(QString,QString,int)), Qt::QueuedConnection);

	downloadButton->hide();
	playButton->hide();
	warn_image_label->hide();
	warning_label->hide();

	titleLabel->setMinimumWidth(100);
	subjectLabel->setMinimumWidth(100);
	warning_label->setMinimumWidth(100);

	small();
	updateItemStatic();
	updateItem();
}

void ChanMsgItem::updateItemStatic()
{
	/* fill in */
#ifdef DEBUG_ITEM
	std::cerr << "ChanMsgItem::updateItemStatic()";
	std::cerr << std::endl;
#endif

	ChannelMsgInfo cmi;

	if (!rsChannels) 
		return;

	if (!rsChannels->getChannelMessage(mChanId, mMsgId, cmi))
		return;

	m_inUpdateItemStatic = true;

	QString title;

	ChannelInfo ci;
	rsChannels->getChannelInfo(mChanId, ci);

	if (!mIsHome)
	{
		title = tr("Channel Feed") + ": ";
		RetroShareLink link;
		link.createChannel(ci.channelId, "");
		title += link.toHtml();
		titleLabel->setText(title);
		RetroShareLink msgLink;
		msgLink.createChannel(cmi.channelId, cmi.msgId);
		subjectLabel->setText(msgLink.toHtml());

		if ((ci.channelFlags & RS_DISTRIB_SUBSCRIBED) || (ci.channelFlags & RS_DISTRIB_ADMIN)) {
			unsubscribeButton->setEnabled(true);
		} else {
			unsubscribeButton->setEnabled(false);
		}
		readButton->hide();
		newLabel->hide();
		copyLinkButton->hide();
	}
	else
	{
		/* subject */
		titleLabel->setText(QString::fromStdWString(cmi.subject));
		subjectLabel->setText(QString::fromStdWString(cmi.msg));

		/* disable buttons: deletion facility not enabled with cache services yet */
		clearButton->setEnabled(false);
		unsubscribeButton->setEnabled(false);
		clearButton->hide();
		unsubscribeButton->hide();
		copyLinkButton->show();

		if ((ci.channelFlags & RS_DISTRIB_SUBSCRIBED) || (ci.channelFlags & RS_DISTRIB_ADMIN)) {
			readButton->setVisible(true);

			uint32_t status = 0;
			rsChannels->getMessageStatus(mChanId, mMsgId, status);

			if ((status & CHANNEL_MSG_STATUS_READ) == 0 || (status & CHANNEL_MSG_STATUS_UNREAD_BY_USER)) {
				readButton->setChecked(true);
				readButton->setIcon(QIcon(":/images/message-state-unread.png"));
			} else {
				readButton->setChecked(false);
				readButton->setIcon(QIcon(":/images/message-state-read.png"));
			}

			if (status & CHANNEL_MSG_STATUS_READ) {
				newLabel->setVisible(false);
				frame->setStyleSheet("QFrame#frame {border: 3px solid #D3D3D3;background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #FFFFFF, stop:1 #F2F2F2);border-radius: 10px;}");

			} else {
				newLabel->setVisible(true);
				frame->setStyleSheet("QFrame#frame {border: 3px solid #82B9F4;background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #F0F8FD, stop:0.8 #E6F2FD, stop: 0.81 #E6F2FD, stop: 1 #D2E7FD);border-radius: 10px;}");
			}
		} else {
			readButton->setVisible(false);
			newLabel->setVisible(false);
		}
	}

	msgLabel->setText(QString::fromStdWString(cmi.msg));
	msgWidget->setVisible(!cmi.msg.empty());

	QDateTime qtime;
	qtime.setTime_t(cmi.ts);
	QString timestamp = qtime.toString("dd.MMMM yyyy hh:mm");
	datetimelabel->setText(timestamp);
	
	{
		std::ostringstream out;
		out << "(" << cmi.count << " Files)";
		filelabel->setText(QString::fromStdString(out.str()) + " " + misc::friendlyUnit(cmi.size));
	}

	if (mFileItems.empty() == false) {
		std::list<SubFileItem *>::iterator it;
		for(it = mFileItems.begin(); it != mFileItems.end(); it++)
		{
			delete(*it);
		}
		mFileItems.clear();
	}

	std::list<FileInfo>::iterator it;
	for(it = cmi.files.begin(); it != cmi.files.end(); it++)
	{
		/* add file */
		SubFileItem *fi = new SubFileItem(it->hash, it->fname, it->path, it->size,
				SFI_STATE_REMOTE | SFI_TYPE_CHANNEL, "");
		mFileItems.push_back(fi);

		QLayout *layout = expandFrame->layout();
		layout->addWidget(fi);
	}

	if(cmi.thumbnail.image_thumbnail != NULL)
	{
		QPixmap thumbnail;
		thumbnail.loadFromData(cmi.thumbnail.image_thumbnail, cmi.thumbnail.im_thumbnail_size,
				"PNG");

		label->setPixmap(thumbnail);
		label->setStyleSheet("QLabel#label{border: 2px solid #D3D3D3;border-radius: 3px;}");
	}

	m_inUpdateItemStatic = false;
}

void ChanMsgItem::setFileCleanUpWarning(uint32_t time_left)
{
	int hours = (int)time_left/3600;
	int minutes = (time_left - hours*3600)%60;

	warning_label->setText(tr("Warning! You have less than %1 hours and %2 minute before this file is delted Consider saving it.").arg(
			QString::number(hours)).arg(QString::number(minutes)));

	QFont warnFont = warning_label->font();
	warnFont.setBold(true);
	warning_label->setFont(warnFont);

	warn_image_label->setVisible(true);
	warning_label->setVisible(true);
	return;

}

void ChanMsgItem::updateItem()
{
	/* fill in */
#ifdef DEBUG_ITEM
	std::cerr << "ChanMsgItem::updateItem()";
	std::cerr << std::endl;
#endif
	int msec_rate = 10000;

	int downloadCount = 0;
	int downloadStartable = 0;
	int playCount = 0;
	int playStartable = 0;
	bool startable;
	bool loopAgain = false;

	/* Very slow Tick to check when all files are downloaded */
	std::list<SubFileItem *>::iterator it;
	for(it = mFileItems.begin(); it != mFileItems.end(); it++)
	{
		SubFileItem *item = *it;

		if (item->isDownloadable(startable)) {
			downloadCount++;
			if (startable) {
				downloadStartable++;
			}
		}
		if (item->isPlayable(startable)) {
			playCount++;
			if (startable) {
				playStartable++;
			}
		}

		if (!item->done())
		{
			/* loop again */
			loopAgain = true;
		}
	}

	if (downloadCount) {
		downloadButton->show();

		if (downloadStartable) {
			downloadButton->setEnabled(true);
		} else {
			downloadButton->setEnabled(false);
		}
	} else {
		downloadButton->hide();
	}
	if (playCount) {
		/* one file is playable */
		playButton->show();

		if (playStartable == 1) {
			playButton->setEnabled(true);
		} else {
			playButton->setEnabled(false);
		}
	} else {
		playButton->hide();
	}

	if (loopAgain) {
		QTimer::singleShot( msec_rate, this, SLOT(updateItem( void ) ));
	}
}

void ChanMsgItem::small()
{
	expandFrame->hide();
}

void ChanMsgItem::toggle()
{
	if (expandFrame->isHidden())
	{
		expandFrame->show();
		expandButton->setIcon(QIcon(QString(":/images/edit_remove24.png")));
		expandButton->setToolTip(tr("Hide"));

		if (!mIsHome) {
			readToggled(false);
		}
	}
	else
	{
		expandFrame->hide();
		expandButton->setIcon(QIcon(QString(":/images/edit_add24.png")));
		expandButton->setToolTip(tr("Expand"));
	}
}

void ChanMsgItem::removeItem()
{
#ifdef DEBUG_ITEM
	std::cerr << "ChanMsgItem::removeItem()";
	std::cerr << std::endl;
#endif
	hide();
	if (mParent)
	{
		mParent->deleteFeedItem(this, mFeedId);
	}
}

/*********** SPECIFIC FUNCTIONS ***********************/

void ChanMsgItem::unsubscribeChannel()
{
#ifdef DEBUG_ITEM
	std::cerr << "ChanMsgItem::unsubscribeChannel()";
	std::cerr << std::endl;
#endif

	if (rsChannels)
	{
                rsChannels->channelSubscribe(mChanId, false, false);
	}
	updateItemStatic();
}

void ChanMsgItem::download()
{
	std::list<SubFileItem *>::iterator it;
	for(it = mFileItems.begin(); it != mFileItems.end(); it++)
	{
		(*it)->download();
	}

	updateItem();
}

void ChanMsgItem::play()
{
	std::list<SubFileItem *>::iterator it;
	for(it = mFileItems.begin(); it != mFileItems.end(); it++)
	{
		bool startable;
		if ((*it)->isPlayable(startable) && startable) {
			(*it)->play();
		}
	}
}

void ChanMsgItem::readToggled(bool checked)
{
	if (m_inUpdateItemStatic) {
		return;
	}

	/* set always as read ... */
	uint32_t statusNew = CHANNEL_MSG_STATUS_READ;
	if (checked) {
		/* ... and as unread by user */
		statusNew |= CHANNEL_MSG_STATUS_UNREAD_BY_USER;
	} else {
		/* ... and as read by user */
		statusNew &= ~CHANNEL_MSG_STATUS_UNREAD_BY_USER;
	}
	rsChannels->setMessageStatus(mChanId, mMsgId, statusNew, CHANNEL_MSG_STATUS_READ | CHANNEL_MSG_STATUS_UNREAD_BY_USER);
}

void ChanMsgItem::channelMsgReadSatusChanged(const QString& channelId, const QString& msgId, int /*status*/)
{
	if (channelId.toStdString() == mChanId && msgId.toStdString() == mMsgId) {
		updateItemStatic();
	}
}

void ChanMsgItem::copyLink()
{
	if (mChanId.empty() || mMsgId.empty()) {
		return;
	}

	ChannelMsgInfo cmi;
	if (rsChannels->getChannelMessage(mChanId, mMsgId, cmi)) {
		RetroShareLink link;
		if (link.createChannel(cmi.channelId, cmi.msgId)) {
			QList<RetroShareLink> urls;
			urls.push_back(link);
			RSLinkClipboard::copyLinks(urls);
		}
	}
}
