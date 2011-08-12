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

#include <QFontDialog>
#include <time.h>

#include "ChatPage.h"
#include "gui/chat/ChatStyle.h"
#include "gui/notifyqt.h"
#include "rsharesettings.h"

#define VARIANT_STANDARD    "Standard"

static QString loadStyleInfo(ChatStyle::enumStyleType type, QListWidget *listWidget, QComboBox *comboBox, QString &styleVariant)
{
    QList<ChatStyleInfo> styles;
    QList<ChatStyleInfo>::iterator style;
    QListWidgetItem *item;
    QListWidgetItem *activeItem = NULL;

    QString stylePath;

    switch (type) {
    case ChatStyle::TYPE_PUBLIC:
        Settings->getPublicChatStyle(stylePath, styleVariant);
        break;
    case ChatStyle::TYPE_PRIVATE:
        Settings->getPrivateChatStyle(stylePath, styleVariant);
        break;
    case ChatStyle::TYPE_HISTORY:
        Settings->getHistoryChatStyle(stylePath, styleVariant);
        break;
    case ChatStyle::TYPE_UNKNOWN:
        return "";
    }

    ChatStyle::getAvailableStyles(type, styles);
    for (style = styles.begin(); style != styles.end(); style++) {
        item = new QListWidgetItem(style->styleName);
        item->setData(Qt::UserRole, qVariantFromValue(*style));
        listWidget->addItem(item);

        if (style->stylePath == stylePath) {
            activeItem = item;
        }
    }

    listWidget->setCurrentItem(activeItem);

    /* now the combobox should be filled */

    int index = comboBox->findText(styleVariant);
    if (index != -1) {
        comboBox->setCurrentIndex(index);
    } else {
        if (comboBox->count()) {
            comboBox->setCurrentIndex(0);
        }
    }
    return stylePath;
}

/** Constructor */
ChatPage::ChatPage(QWidget * parent, Qt::WFlags flags)
    : ConfigPage(parent, flags)
{
    /* Invoke the Qt Designer generated object setup routine */
    ui.setupUi(this);

    /* Hide platform specific features */
#ifdef Q_WS_WIN

#endif
}

/** Saves the changes on this page */
bool
ChatPage::save(QString &/*errmsg*/)
{
    Settings->beginGroup(QString("Chat"));

    Settings->setValue(QString::fromUtf8("Emoteicons_PrivatChat"), ui.checkBox_emoteprivchat->isChecked());
    Settings->setValue(QString::fromUtf8("Emoteicons_GroupChat"), ui.checkBox_emotegroupchat->isChecked());
    Settings->setValue(QString::fromUtf8("GroupChat_History"), ui.checkBox_groupchathistory->isChecked());
    Settings->setValue(QString::fromUtf8("PrivateChat_History"), ui.checkBox_privatechathistory->isChecked());
    Settings->endGroup();

    Settings->setChatScreenFont(fontTempChat.toString());

    Settings->setChatSendMessageWithCtrlReturn(ui.sendMessageWithCtrlReturn->isChecked());

    Settings->setPublicChatHistoryCount(ui.groupchatspinBox->value());
    Settings->setPrivateChatHistoryCount(ui.privatchatspinBox->value());

    ChatStyleInfo info;
    QListWidgetItem *item = ui.publicList->currentItem();
    if (item) {
        info = qVariantValue<ChatStyleInfo>(item->data(Qt::UserRole));
        if (publicStylePath != info.stylePath || publicStyleVariant != ui.publicComboBoxVariant->currentText()) {
            Settings->setPublicChatStyle(info.stylePath, ui.publicComboBoxVariant->currentText());
            NotifyQt::getInstance()->notifyChatStyleChanged(ChatStyle::TYPE_PUBLIC);
        }
    }

    item = ui.privateList->currentItem();
    if (item) {
        info = qVariantValue<ChatStyleInfo>(item->data(Qt::UserRole));
        if (privateStylePath != info.stylePath || privateStyleVariant != ui.privateComboBoxVariant->currentText()) {
            Settings->setPrivateChatStyle(info.stylePath, ui.privateComboBoxVariant->currentText());
            NotifyQt::getInstance()->notifyChatStyleChanged(ChatStyle::TYPE_PRIVATE);
        }
    }

    item = ui.historyList->currentItem();
    if (item) {
        info = qVariantValue<ChatStyleInfo>(item->data(Qt::UserRole));
        if (historyStylePath != info.stylePath || historyStyleVariant != ui.historyComboBoxVariant->currentText()) {
            Settings->setHistoryChatStyle(info.stylePath, ui.historyComboBoxVariant->currentText());
            NotifyQt::getInstance()->notifyChatStyleChanged(ChatStyle::TYPE_HISTORY);
        }
    }

    return true;
}

/** Loads the settings for this page */
void
ChatPage::load()
{
    Settings->beginGroup(QString("Chat"));

    ui.checkBox_emoteprivchat->setChecked(Settings->value(QString::fromUtf8("Emoteicons_PrivatChat"), true).toBool());
    ui.checkBox_emotegroupchat->setChecked(Settings->value(QString::fromUtf8("Emoteicons_GroupChat"), true).toBool());
    ui.checkBox_groupchathistory->setChecked(Settings->value(QString::fromUtf8("GroupChat_History"), true).toBool());
    ui.checkBox_privatechathistory->setChecked(Settings->value(QString::fromUtf8("PrivateChat_History"), true).toBool());

    Settings->endGroup();

    fontTempChat.fromString(Settings->getChatScreenFont());

    ui.sendMessageWithCtrlReturn->setChecked(Settings->getChatSendMessageWithCtrlReturn());

    ui.groupchatspinBox->setValue(Settings->getPublicChatHistoryCount());
    ui.privatchatspinBox->setValue(Settings->getPrivateChatHistoryCount());

    ui.labelChatFontPreview->setText(fontTempChat.rawName());
    ui.labelChatFontPreview->setFont(fontTempChat);

    /* Load styles */
    publicStylePath = loadStyleInfo(ChatStyle::TYPE_PUBLIC, ui.publicList, ui.publicComboBoxVariant, publicStyleVariant);
    privateStylePath = loadStyleInfo(ChatStyle::TYPE_PRIVATE, ui.privateList, ui.privateComboBoxVariant, privateStyleVariant);
    historyStylePath = loadStyleInfo(ChatStyle::TYPE_HISTORY, ui.historyList, ui.historyComboBoxVariant, historyStyleVariant);
}

void ChatPage::on_pushButtonChangeChatFont_clicked()
{
	bool ok;
	QFont font = QFontDialog::getFont(&ok, fontTempChat, this);
	if (ok) {
		fontTempChat = font;
		ui.labelChatFontPreview->setText(fontTempChat.rawName());
		ui.labelChatFontPreview->setFont(fontTempChat);
	}
}

void ChatPage::setPreviewMessages(QString &stylePath, QString styleVariant, QTextBrowser *textBrowser)
{
    ChatStyle style;
    style.setStylePath(stylePath, styleVariant);

    textBrowser->clear();

    QString nameIncoming = "Incoming";
    QString nameOutgoing = "Outgoing";
    QDateTime timestmp = QDateTime::fromTime_t(time(NULL));
    QTextEdit textEdit;
    QString message;

    textEdit.setText(tr("Incoming message in history"));
    message = textEdit.toHtml();
    textBrowser->append(style.formatMessage(ChatStyle::FORMATMSG_HINCOMING, nameIncoming, timestmp, message, CHAT_FORMATTEXT_EMBED_SMILEYS));
    textEdit.setText(tr("Outgoing message in history"));
    message = textEdit.toHtml();
    textBrowser->append(style.formatMessage(ChatStyle::FORMATMSG_HOUTGOING, nameOutgoing, timestmp, message, CHAT_FORMATTEXT_EMBED_SMILEYS));
    textEdit.setText(tr("Incoming message"));
    message = textEdit.toHtml();
    textBrowser->append(style.formatMessage(ChatStyle::FORMATMSG_INCOMING,  nameIncoming, timestmp, message, CHAT_FORMATTEXT_EMBED_SMILEYS));
    textEdit.setText(tr("Outgoing message"));
    message = textEdit.toHtml();
    textBrowser->append(style.formatMessage(ChatStyle::FORMATMSG_OUTGOING,  nameOutgoing, timestmp, message, CHAT_FORMATTEXT_EMBED_SMILEYS));
    textEdit.setText(tr("Outgoing offline message"));
    message = textEdit.toHtml();
    textBrowser->append(style.formatMessage(ChatStyle::FORMATMSG_OOUTGOING,  nameOutgoing, timestmp, message, CHAT_FORMATTEXT_EMBED_SMILEYS));
}

void ChatPage::fillPreview(QListWidget *listWidget, QComboBox *comboBox, QTextBrowser *textBrowser)
{
    QListWidgetItem *item = listWidget->currentItem();
    if (item) {
        ChatStyleInfo info = qVariantValue<ChatStyleInfo>(item->data(Qt::UserRole));

        setPreviewMessages(info.stylePath, comboBox->currentText(), textBrowser);
    } else {
        textBrowser->clear();
    }
}

void ChatPage::on_publicList_currentRowChanged(int currentRow)
{
    if (currentRow != -1) {
        QListWidgetItem *item = ui.publicList->item(currentRow);
        ChatStyleInfo info = qVariantValue<ChatStyleInfo>(item->data(Qt::UserRole));

        QString author = info.authorName;
        if (info.authorEmail.isEmpty() == false) {
            author += " (" + info.authorEmail + ")";
        }
        ui.publicAuthor->setText(author);
        ui.publicDescription->setText(info.styleDescription);

        QStringList variants;
        ChatStyle::getAvailableVariants(info.stylePath, variants);
        ui.publicComboBoxVariant->clear();
        ui.publicComboBoxVariant->setEnabled(variants.size() != 0);
        ui.publicComboBoxVariant->addItems(variants);

        /* try to find "Standard" */
        int index = ui.publicComboBoxVariant->findText(VARIANT_STANDARD);
        if (index != -1) {
            ui.publicComboBoxVariant->setCurrentIndex(index);
        } else {
            ui.publicComboBoxVariant->setCurrentIndex(0);
        }
    } else {
        ui.publicAuthor->clear();
        ui.publicDescription->clear();
        ui.publicComboBoxVariant->clear();
        ui.publicComboBoxVariant->setDisabled(true);
    }

    fillPreview(ui.publicList, ui.publicComboBoxVariant, ui.publicPreview);
}

void ChatPage::on_privateList_currentRowChanged(int currentRow)
{
    if (currentRow != -1) {
        QListWidgetItem *item = ui.privateList->item(currentRow);
        ChatStyleInfo info = qVariantValue<ChatStyleInfo>(item->data(Qt::UserRole));

        QString author = info.authorName;
        if (info.authorEmail.isEmpty() == false) {
            author += " (" + info.authorEmail + ")";
        }
        ui.privateAuthor->setText(author);
        ui.privateDescription->setText(info.styleDescription);

        QStringList variants;
        ChatStyle::getAvailableVariants(info.stylePath, variants);
        ui.privateComboBoxVariant->clear();
        ui.privateComboBoxVariant->setEnabled(variants.size() != 0);
        ui.privateComboBoxVariant->addItems(variants);

        /* try to find "Standard" */
        int index = ui.privateComboBoxVariant->findText(VARIANT_STANDARD);
        if (index != -1) {
            ui.privateComboBoxVariant->setCurrentIndex(index);
        } else {
            ui.privateComboBoxVariant->setCurrentIndex(0);
        }
    } else {
        ui.privateAuthor->clear();
        ui.privateDescription->clear();
        ui.privateComboBoxVariant->clear();
        ui.privateComboBoxVariant->setDisabled(true);
    }

    fillPreview(ui.privateList, ui.privateComboBoxVariant, ui.privatePreview);
}

void ChatPage::on_historyList_currentRowChanged(int currentRow)
{
    if (currentRow != -1) {
        QListWidgetItem *item = ui.historyList->item(currentRow);
        ChatStyleInfo info = qVariantValue<ChatStyleInfo>(item->data(Qt::UserRole));

        QString author = info.authorName;
        if (info.authorEmail.isEmpty() == false) {
            author += " (" + info.authorEmail + ")";
        }
        ui.historyAuthor->setText(author);
        ui.historyDescription->setText(info.styleDescription);

        QStringList variants;
        ChatStyle::getAvailableVariants(info.stylePath, variants);
        ui.historyComboBoxVariant->clear();
        ui.historyComboBoxVariant->setEnabled(variants.size() != 0);
        ui.historyComboBoxVariant->addItems(variants);

        /* try to find "Standard" */
        int index = ui.historyComboBoxVariant->findText(VARIANT_STANDARD);
        if (index != -1) {
            ui.historyComboBoxVariant->setCurrentIndex(index);
        } else {
            ui.historyComboBoxVariant->setCurrentIndex(0);
        }
    } else {
        ui.historyAuthor->clear();
        ui.historyDescription->clear();
        ui.historyComboBoxVariant->clear();
        ui.historyComboBoxVariant->setDisabled(true);
    }

    fillPreview(ui.historyList, ui.historyComboBoxVariant, ui.historyPreview);
}

void ChatPage::on_publicComboBoxVariant_currentIndexChanged(int /*index*/)
{
    fillPreview(ui.publicList, ui.publicComboBoxVariant, ui.publicPreview);
}

void ChatPage::on_privateComboBoxVariant_currentIndexChanged(int /*index*/)
{
    fillPreview(ui.privateList, ui.privateComboBoxVariant, ui.privatePreview);
}

void ChatPage::on_historyComboBoxVariant_currentIndexChanged(int /*index*/)
{
    fillPreview(ui.historyList, ui.historyComboBoxVariant, ui.historyPreview);
}
