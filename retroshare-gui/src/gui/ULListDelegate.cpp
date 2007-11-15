/****************************************************************
 *  RetroShare is distributed under the following license:
 *
 *  Copyright (C) 2006,2007 crypton
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

#include <QModelIndex>
#include <QPainter>
#include <QStyleOptionProgressBarV2>
#include <QProgressBar>
#include <QApplication>

#include "ULListDelegate.h"

ULListDelegate::ULListDelegate(QObject *parent) : QAbstractItemDelegate(parent)
{
	;
}

ULListDelegate::~ULListDelegate(void)
{
	;
}

void ULListDelegate::paint(QPainter * painter, const QStyleOptionViewItem & option, const QModelIndex & index) const
{
	QString byteUnits[4] = {tr("B"), tr("KB"), tr("MB"), tr("GB")};
	QStyleOptionViewItem opt = option;
	QStyleOptionProgressBarV2 newopt;
	QRect pixmapRect;
	QPixmap pixmap;
	qlonglong fileSize;
	double progress, ulspeed, multi;
	QString temp , status;
	qlonglong transferred;

	//set text color
	QVariant value = index.data(Qt::TextColorRole);
	if(value.isValid() && qvariant_cast<QColor>(value).isValid()) {
		opt.palette.setColor(QPalette::Text, qvariant_cast<QColor>(value));
	}
	QPalette::ColorGroup cg = option.state & QStyle::State_Enabled ? QPalette::Normal : QPalette::Disabled;
	if(option.state & QStyle::State_Selected){
		painter->setPen(opt.palette.color(cg, QPalette::HighlightedText));
	} else {
		painter->setPen(opt.palette.color(cg, QPalette::Text));
	}

	// draw the background color
	if(index.column() != UPROGRESS) {
		if(option.showDecorationSelected && (option.state & QStyle::State_Selected)) {
			if(cg == QPalette::Normal && !(option.state & QStyle::State_Active)) {
				cg = QPalette::Inactive;
			}
			painter->fillRect(option.rect, option.palette.brush(cg, QPalette::Highlight));
		} else {
			value = index.data(Qt::BackgroundColorRole);
			if(value.isValid() && qvariant_cast<QColor>(value).isValid()) {
				painter->fillRect(option.rect, qvariant_cast<QColor>(value));
			}
		}
	}
	switch(index.column()) {
		case USIZE:
			fileSize = index.data().toLongLong();
			if(fileSize < 0){
				temp = "Unknown";
			} else {
				multi = 1.0;
				for(int i = 0; i < 5; ++i) {
					if (fileSize < 1024) {
						fileSize = index.data().toLongLong();
						temp.sprintf("%.2f ", fileSize / multi);
						temp += byteUnits[i];
						break;
					}
					fileSize /= 1024;
					multi *= 1024.0;
				}
			}
			painter->drawText(option.rect, Qt::AlignCenter, temp);
			break;
		case UTRANSFERRED:
			transferred = index.data().toLongLong();
			if(transferred < 0){
				temp = "Unknown";
			} else {
				multi = 1.0;
				for(int i = 0; i < 5; ++i) {
					if (transferred < 1024) {
						transferred = index.data().toLongLong();
						temp.sprintf("%.2f ", transferred / multi);
						temp += byteUnits[i];
						break;
					}
					transferred /= 1024;
					multi *= 1024.0;
				}
			}
			painter->drawText(option.rect, Qt::AlignCenter, temp);
			break;
		case ULSPEED:
		    ulspeed = index.data().toDouble();
			temp.clear();
			temp.sprintf("%.2f", ulspeed/1024.);
			temp += " KB/s";
			painter->drawText(option.rect, Qt::AlignCenter, temp);
			break;
		case UPROGRESS:
			progress = index.data().toDouble();
			temp.clear();
			temp.sprintf("%.2f", progress);
			temp += "%";
			newopt.rect = opt.rect;
			//newopt.text = temp;
			newopt.maximum = 100;
			newopt.minimum = 0;
			newopt.progress = (int)progress;
			newopt.state |= QStyle::State_Enabled;
			newopt.textVisible = false;
			QApplication::style()->drawControl(QStyle::CE_ProgressBar, &newopt,
			painter);
        		//We prefer to display text manually to control color/font/boldness
			if (option.state & QStyle::State_Selected){
				opt.palette.setColor(QPalette::Text, QColor("grey"));
				painter->setPen(opt.palette.color(cg, QPalette::Text));
			}
			painter->drawText(option.rect, Qt::AlignCenter, newopt.text);
			break;
		case UNAME:
        		// decoration
			value = index.data(Qt::DecorationRole);
			pixmap = qvariant_cast<QIcon>(value).pixmap(option.decorationSize, option.state & QStyle::State_Enabled ? QIcon::Normal : QIcon::Disabled, option.state & QStyle::State_Open ? QIcon::On : QIcon::Off);
			pixmapRect = (pixmap.isNull() ? QRect(0, 0, 0, 0): QRect(QPoint(0, 0), option.decorationSize));
			if (pixmapRect.isValid()){
				QPoint p = QStyle::alignedRect(option.direction, Qt::AlignLeft, pixmap.size(), option.rect).topLeft();
				painter->drawPixmap(p, pixmap);
			}
			painter->drawText(option.rect.translated(pixmap.size().width(), 0), Qt::AlignLeft, index.data().toString());
			break;
		default:
			painter->drawText(option.rect, Qt::AlignCenter, index.data().toString());
	}
}

QSize ULListDelegate::sizeHint(const QStyleOptionViewItem & option, const QModelIndex & index) const
{
	QVariant value = index.data(Qt::FontRole);
	QFont fnt = value.isValid() ? qvariant_cast<QFont>(value) : option.font;
	QFontMetrics fontMetrics(fnt);
	const QString text = index.data(Qt::DisplayRole).toString();
	QRect textRect = QRect(0, 0, 0, fontMetrics.lineSpacing() * (text.count(QLatin1Char('\n')) + 1));
	return textRect.size();
}

