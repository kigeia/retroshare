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

#include <QMessageBox>

#include "util/misc.h"

#include "CreateForum.h"

#include <retroshare/rsforums.h>

/** Constructor */
CreateForum::CreateForum(QWidget *parent)
: QDialog(parent)
{
  /* Invoke the Qt Designer generated object setup routine */
  ui.setupUi(this);
  
  // connect up the buttons.
  connect( ui.cancelButton, SIGNAL( clicked ( bool ) ), this, SLOT( cancelForum( ) ) );
  connect( ui.createButton, SIGNAL( clicked ( bool ) ), this, SLOT( createForum( ) ) );

  newForum();
}

void  CreateForum::newForum()
{
	/* enforce Public for the moment */
	ui.typePublic->setChecked(true);

	ui.typePrivate->setEnabled(false);
	ui.typeEncrypted->setEnabled(false);

#ifdef RS_RELEASE_VERSION
	ui.typePrivate->setVisible(false);
	ui.typeEncrypted->setVisible(false);
#endif

	ui.msgAnon->setChecked(true);
	//ui.msgAuth->setEnabled(false);

	ui.forumName->clear();
	ui.forumDesc->clear();
}

void  CreateForum::createForum()
{
	QString name = misc::removeNewLine(ui.forumName->text());
	QString desc = ui.forumDesc->toPlainText(); //toHtml();
	uint32_t flags = 0;

	if(name.isEmpty())
	{	/* error message */
		QMessageBox::warning(this, "RetroShare",
							 tr("Please add a Name"),
							 QMessageBox::Ok, QMessageBox::Ok);
		return; //Don't add  a empty name!!
	}
	else

	if (ui.typePublic->isChecked())
	{
		flags |= RS_DISTRIB_PUBLIC;
	}
	else if (ui.typePrivate->isChecked())
	{
		flags |= RS_DISTRIB_PRIVATE;
	}
	else if (ui.typeEncrypted->isChecked())
	{
		flags |= RS_DISTRIB_ENCRYPTED;
	}

	if (ui.msgAuth->isChecked())
	{
		flags |= RS_DISTRIB_AUTHEN_REQ;
	}
	else if (ui.msgAnon->isChecked())
	{
		flags |= RS_DISTRIB_AUTHEN_ANON;
	}

	if (rsForums)
	{
		rsForums->createForum(name.toStdWString(), desc.toStdWString(), flags);
	}

	close();
}


void  CreateForum::cancelForum()
{
	close();
}
