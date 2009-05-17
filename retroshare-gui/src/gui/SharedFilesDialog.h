/****************************************************************
 *  RetroShare is distributed under the following license:
 *
 *  Copyright (C) 2006-2009, RetroShare Team
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

#ifndef _SHAREDFILESDIALOG_H
#define _SHAREDFILESDIALOG_H

#include <QFileDialog>
#include <QMovie>

//#include <config/rsharesettings.h>

#include "mainpage.h"
#include "ui_SharedFilesDialog.h"

#include "rsiface/rstypes.h"
#include "rsiface/RemoteDirModel.h"

class SharedFilesDialog : public MainPage 
{
  Q_OBJECT

public:
  /** Default Constructor */
  SharedFilesDialog(QWidget *parent = 0);
  /** Default Destructor */



private slots:

	/* For handling the model updates */
  void  preModDirectories(bool update_local);
  void  postModDirectories(bool update_local);

  void checkUpdate();
  void forceCheck();

  /** Create the context popup menu and it's submenus */
  void shareddirtreeviewCostumPopupMenu( QPoint point );
  
  void sharedDirTreeWidgetContextMenu( QPoint point );
  
  void downloadRemoteSelected();
//  void addMsgRemoteSelected();

  //void showFrame(bool show);


//  void recommendfile();
  void playselectedfiles();
  void openfile();
  void openfolder();

//  void recommendFileSetOnly();
  void recommendFilesTo( std::string rsid );
  void recommendFilesToMsg( std::string rsid );
  void runCommandForFile();
  void tryToAddNewAssotiation();

signals:
  void playFiles(QStringList files);

private:
  //now context menu are created again every time theu are called ( in some
  //slots.. Maybe it's not good...
  //** Define the popup menus for the Context menu */
  //QMenu* contextMnu;
  
  //QMenu* contextMnu2;
  
  /** Defines the actions for the context menu for QTreeView */
  QAction* downloadAct;
  QAction* addMsgAct;
  
  /** Defines the actions for the context menu for QTreeWidget */
  QAction* openfileAct;
  QAction* openfolderAct;
  
  
  QTreeView *shareddirtreeview;
  QMovie *movie;

  /** Qt Designer generated object */
  Ui::SharedFilesDialog ui;

  /* RemoteDirModel */
  RemoteDirModel *model;
  RemoteDirModel *localModel;

  QString currentCommand;
  QString currentFile;
  
  QAction* fileAssotiationAction(const QString fileName);
};

#endif

