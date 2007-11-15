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


#ifndef _CHAN_MSG_DIALOG_H
#define _CHAN_MSG_DIALOG_H

#include <QMainWindow>

#include "ui_ChanMsgDialog.h"

class ChanMsgDialog : public QMainWindow 
{
  Q_OBJECT

public:
  /** Default Constructor */

  ChanMsgDialog(bool isMsg, QWidget *parent = 0, Qt::WFlags flags = 0);
  /** Default Destructor */

void  newMsg();

	/* worker fns */
void  insertSendList(); /* for Msgs */
void  insertChannelSendList(); /* for Channels */
void  insertFileList(); /* for Both */
void  insertTitleText(std::string title);
void  insertMsgText(std::string msg);

public slots:

	/* actions to take.... */
void  sendMessage();
void  cancelMessage();

private slots:

  /** Create the context popup menu and it's submenus */
  void channelstreeViewCostumPopupMenu( QPoint point );
   
  /** Defines the context menu functions*/
  void deletechannel();
  void createchannelmsg();


	/* for toggling flags */
  void togglePersonItem( QTreeWidgetItem *item, int col );
  void toggleChannelItem( QTreeWidgetItem *item, int col );
  void toggleRecommendItem( QTreeWidgetItem *item, int col );

 
private:

   /** Define the popup menus for the Context menu */
  QMenu* contextMnu;
  
   /** Defines the actions for the context menu */
  QAction* deletechannelAct;
  QAction* createchannelmsgAct;

  QTreeView *channelstreeView;

  bool mIsMsg; /* different behaviour for Msg or ChanMsg */

  /** Qt Designer generated object */
  Ui::ChanMsgDialog ui;

};

#endif

