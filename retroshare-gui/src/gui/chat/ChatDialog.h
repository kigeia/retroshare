/****************************************************************
 *
 *  RetroShare is distributed under the following license:
 *
 *  Copyright (C) 2011, RetroShare Team
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

#ifndef CHATDIALOG_H
#define CHATDIALOG_H

#include <QWidget>
#include <retroshare/rsmsgs.h>

class ChatWidget;
class RSStyle;

class ChatDialog : public QWidget
{
	Q_OBJECT

public:
	static ChatDialog *getExistingChat(const std::string &peerId);
	static ChatDialog *getChat(const std::string &peerId, uint chatflags);
	static void cleanupChat();
	static void chatFriend(const std::string &peerId);
	static void closeChat(const std::string &peerId);
	static void chatChanged(int list, int type);

	virtual bool canClose() { return true; }
	virtual void showDialog(uint chatflags) {}

	virtual ChatWidget *getChatWidget() = 0;
	virtual bool hasPeerStatus() = 0;

	void addToParent(QWidget *newParent);
	void removeFromParent(QWidget *oldParent);

	std::string getPeerId() { return peerId; }
	QString getTitle();
	bool hasNewMessages();
	bool isTyping();

	bool setStyle();
	const RSStyle *getStyle();

	void insertChatMsgs();
	int getPeerStatus();

	void focusDialog();

signals:
	void dialogClose(ChatDialog *dialog);
	void infoChanged(ChatDialog *dialog);
	void newMessage(ChatDialog *dialog);

private slots:
	void chatInfoChanged(ChatWidget*);
	void chatNewMessage(ChatWidget*);

protected:
	explicit ChatDialog(QWidget *parent = 0, Qt::WFlags flags = 0);
	virtual ~ChatDialog();

	virtual void init(const std::string &peerId, const QString &peerName);
	virtual void onChatChanged(int list, int type) {}

	virtual void addIncomingChatMsg(const ChatInfo& info) = 0;

	std::string peerId;
};

#endif // CHATDIALOG_H
