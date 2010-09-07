/****************************************************************
 *  RetroShare is distributed under the following license:
 *
 *  Copyright (C) 2006 - 2010 RetroShare Team
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

#ifndef _CHATPAGE_H
#define _CHATPAGE_H

#include "configpage.h"
#include "ui_ChatPage.h"

class ChatPage : public ConfigPage
{
  Q_OBJECT

  public:
      /** Default Constructor */
      ChatPage(QWidget * parent = 0, Qt::WFlags flags = 0);
      /** Default Destructor */
      ~ChatPage() {}

      /** Saves the changes on this page */
      bool save(QString &errmsg);
      /** Loads the settings for this page */
      void load();

      bool emotePrivatChat() const;
      bool emoteGroupChat() const;
      bool groupchatHistory() const;

      QFont fontTempChat;

  private slots:
      void on_pushButtonChangeChatFont_clicked();
      void on_publicList_currentRowChanged(int currentRow);
      void on_privateList_currentRowChanged(int currentRow);
      void on_historyList_currentRowChanged(int currentRow);

  private:
      void closeEvent (QCloseEvent * event);
      void setPreviewMessages(QString &stylePath, QTextBrowser *textBrowser);

      QString publicStylePath;
      QString privateStylePath;
      QString historyStylePath;

      /** Qt Designer generated object */
      Ui::ChatPage ui;
};

#endif

