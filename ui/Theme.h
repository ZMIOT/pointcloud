#pragma once
#include <QApplication>
#include <QColor>
#include <QFontDatabase>
#include <QPalette>
#include <QSettings>
#include <QStyle>
#include <QWidget>
#include <QMap>
#include <algorithm>

// Shared presentation tokens only; algorithm/SDK interfaces have no UI dependency.
namespace workbench {
struct Colors {
    QString background, surface, raised, text, muted, outline, separator;
    QString interactive, onInteractive, selection, error, errorSurface, warning, warningSurface, success, focus;
};
inline Colors colors(bool dark) {
    // 深色工作区参考 ui/stitch_qt/screen.png；保留原有亮色主题入口。
    if(dark) return {"#0D121B","#141D2C","#1E293B","#E8EEF7","#94A3B8","#3B4B61","#253245",
                     "#17A8DC","#FFFFFF","#1D354B","#FFB4AB","#43272C","#FBBF24","#3B321C","#34D399","#50CFFF"};
    return {"#F1F4F8","#FFFFFF","#E8EEF5","#172438","#52627A","#7D8CA1","#D8E0EB",
            "#1758BD","#FFFFFF","#E3EDFC","#A82424","#FFF0EF","#8A5600","#FFF4D6","#17643B","#1758BD"};
}
inline void repolish(QWidget* widget) {
    widget->style()->unpolish(widget); widget->style()->polish(widget); widget->update();
}
inline void setRole(QWidget* widget,const char* role) { widget->setProperty("role",role); repolish(widget); }
inline void applyTheme(QApplication& app,bool dark) {
    const auto c=colors(dark);
    app.setProperty("workbench.dark",dark);
    QPalette p;
    p.setColor(QPalette::Window,QColor(c.background)); p.setColor(QPalette::WindowText,QColor(c.text));
    p.setColor(QPalette::Base,QColor(c.surface)); p.setColor(QPalette::AlternateBase,QColor(c.raised));
    p.setColor(QPalette::Text,QColor(c.text)); p.setColor(QPalette::Button,QColor(c.surface));
    p.setColor(QPalette::ButtonText,QColor(c.text)); p.setColor(QPalette::Highlight,QColor(c.interactive));
    p.setColor(QPalette::HighlightedText,QColor(c.onInteractive)); p.setColor(QPalette::Link,QColor(c.interactive));
    p.setColor(QPalette::ToolTipBase,QColor(c.surface)); p.setColor(QPalette::ToolTipText,QColor(c.text));
    p.setColor(QPalette::Disabled,QPalette::Text,QColor(c.muted)); p.setColor(QPalette::Disabled,QPalette::ButtonText,QColor(c.muted));
    p.setColor(QPalette::Disabled,QPalette::WindowText,QColor(c.muted));
    app.setPalette(p);
    QString css=QStringLiteral(R"(
        QMainWindow, QDialog { background: @background; }
        QWidget { color: @text; font-family: '@fontFamily'; font-size: @bodypt; }
        QFrame[role="surface"], QWidget[role="surface"] { background: @surface; }
        QWidget[role="viewport"] { background: @background; }
        QWidget[role="emptyCard"] { background: @surface; border: 1px solid @separator; border-radius: 16px; }
        QWidget[role="viewHeader"] { background: @surface; border-bottom: 1px solid @separator; }
        QStackedWidget[role="emptyWell"] { background: @background; border: 1px solid @separator; border-radius: 7px; }
        QLabel[role="viewTitle"], QLabel[role="accent"] { color: @interactive; font-weight: 600; }
        QLabel[role="emptyIcon"] { color: @interactive; background: @selection; border: 1px solid @outline; border-radius: 16px; }
        QLabel { background: transparent; }
        QLabel[role="muted"] { color: @muted; }
        QLabel[role="diagnosticNotice"] { color: @muted; padding: 3px 6px; }
        QLabel[role="diagnosticNotice"][state="critical"] { color: @error; background: @errorSurface; border-radius: 5px; font-weight: 600; }
        QLabel[role="diagnosticNotice"][state="warning"] { color: @warning; background: @warningSurface; border-radius: 5px; font-weight: 600; }
        QLabel[role="diagnosticNotice"][state="info"] { color: @success; font-weight: 600; }
        QToolButton[role="diagnosticDetails"] { min-height: 18px; padding: 2px 6px; }
        QLabel[role="heading"] { font-weight: 600; font-size: @headingpt; }
        QLabel[role="title"] { font-weight: 600; font-size: @titlept; }
        QLabel[role="status"] { color: @success; background: transparent; padding: 2px 8px; }
        QLabel[state="error"], QLabel[role="error"] { color: @error; background: @errorSurface; border-radius: 5px; padding: 8px; }
        QLabel[state="success"] { color: @success; }
        QToolBar { background: @surface; border: none; border-bottom: 1px solid @separator; spacing: 5px; padding: 4px 10px; }
        QToolBar::separator { background: @separator; width: 1px; margin: 6px 2px; }
        QPushButton, QToolButton { background: @raised; border: 1px solid @outline; border-radius: 6px; padding: 4px 8px; min-height: 22px; }
        QPushButton:hover, QToolButton:hover { background: @raised; }
        QPushButton:pressed, QToolButton:pressed, QToolButton:checked { background: @selection; border-color: @interactive; }
        QPushButton[role="primary"], QToolButton[role="primary"] { background: qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0 @interactive,stop:1 #2563EB); color: @onInteractive; border-color: @interactive; font-weight: 600; }
        QPushButton[role="primary"]:hover, QToolButton[role="primary"]:hover { border: 2px solid @focus; padding: 3px 7px; }
        QPushButton:disabled, QToolButton:disabled { background: @raised; color: @muted; border-color: @separator; }
        QPushButton:focus, QToolButton:focus { border: 2px solid @focus; padding: 3px 7px; }
        QToolBar QToolButton { min-height: 20px; padding: 3px 7px; }
        QToolBar QToolButton:focus { padding: 2px 6px; }
        QLineEdit, QComboBox, QAbstractSpinBox, QPlainTextEdit { background: @surface; border: 1px solid @outline; border-radius: 4px; padding: 4px 7px; min-height: 20px; selection-background-color: @interactive; selection-color: @onInteractive; }
        QAbstractSpinBox { padding-right: 22px; }
        QAbstractSpinBox::up-button { subcontrol-origin: border; subcontrol-position: top right; width: 22px; border-left: 1px solid @outline; border-bottom: 1px solid @separator; }
        QAbstractSpinBox::down-button { subcontrol-origin: border; subcontrol-position: bottom right; width: 22px; border-left: 1px solid @outline; }
        QAbstractSpinBox::up-arrow { image: url(:/workbench/up-@theme.svg); width: 12px; height: 12px; }
        QAbstractSpinBox::down-arrow { image: url(:/workbench/down-@theme.svg); width: 12px; height: 12px; }
        QComboBox { padding-right: 26px; }
        QLineEdit:focus, QComboBox:focus, QAbstractSpinBox:focus, QPlainTextEdit:focus { border: 2px solid @focus; padding: 3px 6px; }
        QAbstractSpinBox:focus { padding-right: 21px; }
        QComboBox:focus { padding-right: 25px; }
        QLineEdit:disabled, QComboBox:disabled, QAbstractSpinBox:disabled { background: @raised; color: @muted; border-color: @separator; }
        QCheckBox { spacing: 6px; min-height: 24px; padding: 1px; }
        QCheckBox::indicator, QListWidget::indicator { width: 16px; height: 16px; border: 1px solid @outline; border-radius: 3px; background: @surface; }
        QCheckBox::indicator:checked, QListWidget::indicator:checked { image: url(:/workbench/check-@theme.svg); background: @interactive; border-color: @interactive; }
        QCheckBox::indicator:disabled { background: @raised; border-color: @separator; }
        QCheckBox:focus { background: @selection; border: 1px solid @focus; border-radius: 4px; padding: 0px; }
        QGroupBox { background: @background; border: 1px solid @separator; border-radius: 8px; margin-top: 12px; padding: 12px 10px 8px; }
        QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; left: 10px; padding: 0 5px; font-weight: 600; color: @interactive; }
        QScrollArea, QScrollArea > QWidget > QWidget { border: none; background: @surface; }
        QDockWidget { background: @surface; }
        QDockWidget::title { background: @surface; padding: 6px 10px; border-bottom: 1px solid @separator; font-weight: 600; }
        QMainWindow::separator { background: @background; width: 6px; height: 6px; }
        QMainWindow::separator:hover, QSplitter::handle:hover { background: @outline; }
        QListWidget, QTreeWidget { background: @background; border: 1px solid @separator; border-radius: 7px; outline: 0; }
        QListWidget::item { padding: 6px 8px; min-height: 20px; border-bottom: 1px solid @separator; }
        QListWidget::item:selected { background: @selection; color: @text; }
        QListWidget::item:hover { background: @raised; }
        QListWidget:focus { border: 2px solid @focus; }
        QTabWidget::pane { border: 1px solid @separator; background: @surface; }
        QTabBar::tab { background: @background; color: @muted; padding: 9px 14px; border-bottom: 3px solid transparent; }
        QTabBar::tab:selected { background: @surface; color: @text; border-bottom-color: @interactive; }
        QTabBar::tab:focus { border: 1px solid @focus; }
        QMenuBar { background: @surface; border-bottom: 1px solid @separator; }
        QMenuBar::item { padding: 7px 12px; background: transparent; }
        QMenuBar::item:selected, QMenu::item:selected { background: @selection; color: @text; }
        QMenu { background: @surface; border: 1px solid @outline; padding: 6px; }
        QMenu::item { padding: 8px 24px; }
        QStatusBar { background: @surface; border-top: 1px solid @separator; padding: 2px 8px; }
        QStatusBar::item { border: none; }
        QProgressBar { border: 1px solid @outline; border-radius: 4px; background: @raised; min-height: 18px; text-align: center; }
        QProgressBar::chunk { background: @interactive; }
        QToolTip { background: @surface; color: @text; border: 1px solid @outline; padding: 6px; }
        QScrollBar:vertical { background: @background; width: 12px; margin: 0; }
        QScrollBar::handle:vertical { background: @outline; min-height: 28px; border-radius: 5px; margin: 2px; }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
        QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: transparent; }
    )");
    const QMap<QString,QString> values={{"background",c.background},{"surface",c.surface},{"raised",c.raised},
        {"text",c.text},{"muted",c.muted},{"outline",c.outline},{"separator",c.separator},{"interactive",c.interactive},{"theme",dark?"dark":"light"},
        {"onInteractive",c.onInteractive},{"selection",c.selection},{"error",c.error},{"errorSurface",c.errorSurface},
        {"warning",c.warning},{"warningSurface",c.warningSurface},
        {"success",c.success},{"focus",c.focus},{"fontFamily",app.font().family()},{"bodypt",QString::number(app.font().pointSizeF(),'f',1)+"pt"},{"headingpt",QString::number(app.font().pointSizeF()*1.125,'f',1)+"pt"},
        {"titlept",QString::number(app.font().pointSizeF()*1.266,'f',1)+"pt"}};
    // Longest tokens first: @error must not replace the prefix of @errorSurface.
    auto keys=values.keys(); std::sort(keys.begin(),keys.end(),[](const QString& a,const QString& b) {return a.size()>b.size();});
    for(const auto& key:keys) css.replace("@"+key,values[key]);
    app.setStyleSheet(css);
}
inline void initializeAppearance(QApplication& app,double fontScale=1) {
    app.setStyle("Fusion");
    auto font=QFontDatabase::systemFont(QFontDatabase::GeneralFont);
    // Windows' legacy MS Shell Dlg alias can resolve Chinese QSS text to SimSun.
    // Prefer the installed Windows UI family; keep the platform font elsewhere.
#ifdef Q_OS_WIN
    if(QFontDatabase().families().contains(QStringLiteral("Microsoft YaHei UI"))) font.setFamily(QStringLiteral("Microsoft YaHei UI"));
#endif
    font.setPointSizeF(std::max(10.5,font.pointSizeF())*fontScale); app.setFont(font);
    applyTheme(app,QSettings().value("appearance/dark",true).toBool());
}
}
