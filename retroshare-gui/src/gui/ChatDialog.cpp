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
#include <QtGui>

#include "rshare.h"
#include "ChatDialog.h"

#include "rsiface/rsiface.h"
#include "chat/PopupChatDialog.h"
#include <sstream>

#include <QTextCodec>
#include <QTextEdit>
#include <QToolBar>
#include <QTextCursor>
#include <QTextList>

#include <QContextMenuEvent>
#include <QMenu>
#include <QCursor>
#include <QPoint>
#include <QMouseEvent>
#include <QPixmap>
#include <QHeaderView>

/** Constructor */
ChatDialog::ChatDialog(QWidget *parent)
: MainPage(parent)
{
  /* Invoke the Qt Designer generated object setup routine */
  ui.setupUi(this);

  connect(ui.lineEdit, SIGNAL(returnPressed( ) ), this, SLOT(sendMsg( ) ));
  
  connect(ui.colorChatButton, SIGNAL(clicked()), this, SLOT(setColor()));
  
  connect(ui.textboldChatButton, SIGNAL(clicked()), this, SLOT(textBold()));
  
  connect(ui.textunderlineChatButton, SIGNAL(clicked()), this, SLOT(textUnderline()));
  
  connect(ui.textitalicChatButton, SIGNAL(clicked()), this, SLOT(textItalic()));
  
  connect( ui.msgSendList, SIGNAL( customContextMenuRequested( QPoint ) ), this, SLOT( msgSendListCostumPopupMenu( QPoint ) ) );

//  connect(ui.msgSendList, SIGNAL(itemChanged( QTreeWidgetItem *, int ) ), 
//  this, SLOT(toggleSendItem( QTreeWidgetItem *, int ) ));

  loadInitMsg();
  
  /* hide the Tree +/- */
  ui.msgSendList -> setRootIsDecorated( false );
  
  /* to hide the header  */
  ui.msgSendList->header()->hide();

  /* Hide platform specific features */
#ifdef Q_WS_WIN

#endif
}

void ChatDialog::msgSendListCostumPopupMenu( QPoint point )
{

      QMenu contextMnu( this );
      QMouseEvent *mevent = new QMouseEvent( QEvent::MouseButtonPress, point, Qt::RightButton, Qt::RightButton, Qt::NoModifier );

      privchatAct = new QAction(QIcon(), tr( "Chat" ), this );
      connect( privchatAct , SIGNAL( triggered() ), this, SLOT( privchat() ) );
      
      contextMnu.clear();
      contextMnu.addAction( privchatAct);
      contextMnu.exec( mevent->globalPos() );
}
int     ChatDialog::loadInitMsg()
{
	std::ostringstream out;

	//out << std::endl;
	//out << std::endl;
	//out << std::endl;
	out << "      Welcome to:";
	out << "<br>" << std::endl;
	out << "      Retroshare's group chat. <br>";

	QString txt = QString::fromStdString(out.str());
	ui.msgText->setHtml(txt);

	return 1;
}



void ChatDialog::insertChat()
{
        rsiface->lockData(); /* Lock Interface */

	        /* get a link to the table */
        QTextEdit *msgWidget = ui.msgText;
	std::list<ChatInfo> newchat = rsiface->getChatNew();
	std::list<ChatInfo>::iterator it;

        rsiface->unlockData(); /* Unlock Interface */

static  std::string lastChatName("");
static  int         lastChatTime = 0;


	/* determine how many spaces to add */
	int n = msgWidget->width();
	/* now spaces = (width - txt width) / (pixel / space)
	 */

	//std::cerr << "Width is : " << n << std::endl;
	n -= 256; /* 220 pixels for name */
	if (n > 0)
	{
		n = 2 + n / 10;
	}
	else
	{
		n = 1;
	}
	n = 1 + n / 2; /* shrink it! */

	//std::cerr << "Space count : " << n << std::endl;

	std::string spaces(" ");


	/* add in lines at the bottom */
	int ts = time(NULL);
	for(it = newchat.begin(); it != newchat.end(); it++)
	{
		/* are they private? */
		if (it->chatflags & RS_CHAT_PRIVATE)
		{
			PopupChatDialog *pcd = getPrivateChat(it->rsid, it->name, true);
			pcd->addChatMsg(&(*it));
			continue;
		}

		std::ostringstream out;
		QString currenttxt = msgWidget->toHtml();

		if ((it->name == lastChatName) && (ts - lastChatTime < 60))
		{
			/* no name */
		}
		else
		{
#if defined(Q_OS_WIN)
			/* nothing */
			//out << "<br>" << std::endl;
#else
			out << "<br>" << std::endl;
#endif
			for(int i = 0; i < n; i++)
			{
				out << spaces; 
			}
            		QString timestamp = "(" + QDateTime::currentDateTime().toString("hh:mm:ss") + ") ";
            		QString name = QString::fromStdString(it->name);
            		//QString line = "<span style=\"color:#1D84C9\">" + timestamp +
                       	//	                  "   " + name + "</span> \n<br>";
            		QString line = "<span style=\"color:#1D84C9\"><strong>" + timestamp +
                       		                  "   " + name + "</strong></span> \n<br>";

                	out << line.toStdString();
		}

		out << it -> msg;

	        /* This might be WIN32 only - or maybe Qt4.2.2 only - but need it for windows at the mom */
#if defined(Q_OS_WIN)
		//out << "<br>";
		//out << "<br>" << std::endl;

		out << std::endl;
#else
		out << std::endl;
#endif

		lastChatName = it -> name;
		lastChatTime = ts;

		/* add it everytime */
		QString extra = QString::fromStdString(out.str());
		currenttxt += extra;

		msgWidget->setHtml(currenttxt);

		//std::cerr << " Added Text: " << std::endl;
		//std::cerr << out.str() << std::endl;
		QScrollBar *qsb =  msgWidget->verticalScrollBar();
		qsb -> setValue(qsb->maximum());
	}
}




void ChatDialog::sendMsg()
{
        QLineEdit *lineWidget = ui.lineEdit;

	ChatInfo ci;
	ci.msg = lineWidget->text().toStdString();
	ci.chatflags = RS_CHAT_PUBLIC;

	rsicontrol -> ChatSend(ci);
	lineWidget -> setText(QString(""));

	/* redraw send list */
	insertSendList();


}

void  ChatDialog::insertSendList()
{
        rsiface->lockData(); /* Lock Interface */

        std::map<RsCertId,NeighbourInfo>::const_iterator it;
        const std::map<RsCertId,NeighbourInfo> &friends =
                                rsiface->getFriendMap();

        /* get a link to the table */
        QTreeWidget *sendWidget = ui.msgSendList;

        /* remove old items ??? */
	sendWidget->clear();
	sendWidget->setColumnCount(1);

        QList<QTreeWidgetItem *> items;
	for(it = friends.begin(); it != friends.end(); it++)
	{
		/* if offline, don't add */
		if ((it -> second.connectString == "Online" ) ||
			(it -> second.connectString == "Yourself"))
		{
			/* ok */
		}
		else
		{
			continue;
		}

		/* make a widget per friend */
           	QTreeWidgetItem *item = new QTreeWidgetItem((QTreeWidget*)0);


		/* add all the labels */
		/* (0) Person */
		item -> setText(0, QString::fromStdString(it->second.name));
		/* () Org */
		//item -> setText(1, QString::fromStdString(it->second.org));
		/* () Location */
		//item -> setText(2, QString::fromStdString(it->second.loc));
		/* () Country */
		//item -> setText(3, QString::fromStdString(it->second.country));
		/*{
			std::ostringstream out;
			out << it->second.id;
			item -> setText(4, QString::fromStdString(out.str()));
		}*/

		//item -> setText(5, "Friend");

		//item -> setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
		item -> setFlags(Qt::ItemIsUserCheckable);

		item -> setCheckState(0, Qt::Checked);
		/**** NOT SELECTABLE AT THE MOMENT 
		if (it ->second.inChat)
		{
			item -> setCheckState(0, Qt::Checked);
		}
		else
		{
			item -> setCheckState(0, Qt::Unchecked);
		}
		************/


		/* add to the list */
		items.append(item);
	}

	/* add the items in! */
	sendWidget->insertTopLevelItems(0, items);

	rsiface->unlockData(); /* UnLock Interface */

	sendWidget->update(); /* update display */
}


/* to toggle the state */


void ChatDialog::toggleSendItem( QTreeWidgetItem *item, int col )
{
	std::cerr << "ToggleSendItem()" << std::endl;

	/* extract id */
	std::string id = (item -> text(4)).toStdString();

	/* get state */
	bool inChat = (Qt::Checked == item -> checkState(0)); /* alway column 0 */

	/* call control fns */

	rsicontrol -> SetInChat(id, inChat);
	return;
}

void ChatDialog::setColor()
{
    QColor col = QColorDialog::getColor(Qt::green, this);
    if (col.isValid()) {

        ui.colorChatButton->setPalette(QPalette(col));
        QTextCharFormat fmt;
        fmt.setForeground(col);
        mergeFormatOnWordOrSelection(fmt);
        colorChanged(col);
    }
}

void ChatDialog::textBold()
{
    QTextCharFormat fmt;
    fmt.setFontWeight(ui.textboldChatButton->isChecked() ? QFont::Bold : QFont::Normal);
    mergeFormatOnWordOrSelection(fmt);
}

void ChatDialog::textUnderline()
{
    QTextCharFormat fmt;
    fmt.setFontUnderline(ui.textunderlineChatButton->isChecked());
    mergeFormatOnWordOrSelection(fmt);
}

void ChatDialog::textItalic()
{
    QTextCharFormat fmt;
    fmt.setFontItalic(ui.textitalicChatButton->isChecked());
    mergeFormatOnWordOrSelection(fmt);
}

void ChatDialog::currentCharFormatChanged(const QTextCharFormat &format)
{
    fontChanged(format.font());
    colorChanged(format.foreground().color());
}
   
void ChatDialog::mergeFormatOnWordOrSelection(const QTextCharFormat &format)
{
    QTextCursor cursor = ui.msgText->textCursor();
    if (!cursor.hasSelection())
        cursor.select(QTextCursor::WordUnderCursor);
    cursor.mergeCharFormat(format);
    ui.msgText->mergeCurrentCharFormat(format);
}

void ChatDialog::fontChanged(const QFont &f)
{
    //comboFont->setCurrentIndex(comboFont->findText(QFontInfo(f).family()));
    //comboSize->setCurrentIndex(comboSize->findText(QString::number(f.pointSize())));
    ui.textboldChatButton->setChecked(f.bold());
    ui.textunderlineChatButton->setChecked(f.italic());
    ui.textitalicChatButton->setChecked(f.underline());
}



void ChatDialog::colorChanged(const QColor &c)
{
    QPixmap pix(16, 16);
    pix.fill(c);
    ui.colorChatButton->setIcon(pix);
}


void ChatDialog::privchat()
{

}



PopupChatDialog *ChatDialog::getPrivateChat(std::string id, std::string name, bool show)
{
   /* see if it exists already */
   PopupChatDialog *popupchatdialog = NULL;

   std::map<std::string, PopupChatDialog *>::iterator it;
   if (chatDialogs.end() != (it = chatDialogs.find(id)))
   {
   	/* exists already */
   	popupchatdialog = it->second;
   }
   else 
   {
   	popupchatdialog = new PopupChatDialog(id, name);
	chatDialogs[id] = popupchatdialog;
   }

   if (show)
   {
   	popupchatdialog->show();
   }

   return popupchatdialog;

}

void ChatDialog::clearOldChats()
{
	/* nothing yet */

}


