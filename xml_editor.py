#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import tempfile
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QTextEdit,
    QFileDialog,
    QMessageBox,
    QLabel,
    QSplitter,
    QShortcut,
    QDesktopWidget
)
from PyQt5.QtCore import Qt, QSize, pyqtSignal
from PyQt5.QtGui import QFont, QKeySequence, QTextCharFormat, QColor, QSyntaxHighlighter


class XMLHighlighter(QSyntaxHighlighter):
    """XML syntax highlighter for the editor"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self.highlighting_rules = []
        
        # XML tags
        tag_format = QTextCharFormat()
        tag_format.setForeground(QColor("#0000FF"))  # Blue
        tag_format.setFontWeight(QFont.Bold)
        self.highlighting_rules.append((r'<[/]?[a-zA-Z0-9_]+[^>]*>', tag_format))
        
        # XML attributes
        attribute_format = QTextCharFormat()
        attribute_format.setForeground(QColor("#FF0000"))  # Red
        self.highlighting_rules.append((r'[a-zA-Z0-9_]+(?=\s*=)', attribute_format))
        
        # XML attribute values
        value_format = QTextCharFormat()
        value_format.setForeground(QColor("#008000"))  # Green
        self.highlighting_rules.append((r'"[^"]*"', value_format))
        
        # XML comments
        comment_format = QTextCharFormat()
        comment_format.setForeground(QColor("#808080"))  # Gray
        self.highlighting_rules.append((r'<!--[^<>]*-->', comment_format))
    
    def highlightBlock(self, text):
        import re
        
        for pattern, format in self.highlighting_rules:
            for match in re.finditer(pattern, text):
                start = match.start()
                length = match.end() - match.start()
                self.setFormat(start, length, format)


class XMLEditor(QMainWindow):
    """XML Editor window for editing URDF files"""
    
    def __init__(self, file_path=None, update_callback=None):
        super().__init__()
        self.file_path = file_path
        self.update_callback = update_callback
        self.init_ui()
        
        if file_path:
            self.load_file(file_path)
    
    def init_ui(self):
        """Initialize the user interface"""
        # Set window size
        window_width = 1200
        window_height = 800
        
        # Get screen size and calculate center position
        screen = QDesktopWidget().availableGeometry()
        x = (screen.width() - window_width) // 2
        y = (screen.height() - window_height) // 2
        
        # Set window geometry to be centered on screen
        self.setGeometry(x, y, window_width, window_height)
        # Create central widget and main layout
        central_widget = QWidget()
        main_layout = QVBoxLayout(central_widget)
        
        # Create toolbar
        toolbar_layout = QHBoxLayout()
        
        # Create save as button
        self.btn_save_as = QPushButton("Save As")
        self.btn_save_as.clicked.connect(self.save_file_as)
        toolbar_layout.addWidget(self.btn_save_as)
        
        # Create update button
        self.btn_update = QPushButton("Update")
        self.btn_update.clicked.connect(self.update_model)
        self.btn_update.setToolTip("Update the model in the viewer without saving the file")
        toolbar_layout.addWidget(self.btn_update)
        
        # Add spacer to push the file path label to the right
        toolbar_layout.addStretch()
        
        # Create file path label
        self.file_path_label = QLabel()
        toolbar_layout.addWidget(self.file_path_label)
        
        # Add toolbar to main layout
        main_layout.addLayout(toolbar_layout)
        
        # Create text editor
        self.text_edit = QTextEdit()
        self.text_edit.setFont(QFont("Courier New", 10))
        self.text_edit.setLineWrapMode(QTextEdit.NoWrap)
        
        # Apply syntax highlighting
        self.highlighter = XMLHighlighter(self.text_edit.document())
        
        # Add text editor to main layout
        main_layout.addWidget(self.text_edit)
        
        # Set central widget
        self.setCentralWidget(central_widget)
        
        # Create keyboard shortcuts
        self.shortcut_save = QShortcut(QKeySequence("Ctrl+S"), self)
        self.shortcut_save.activated.connect(self.save_file_as)
        
        # Update file path label
        self.update_file_path_label()
    
    def load_file(self, file_path):
        """Load a file into the editor"""
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                content = file.read()
                self.text_edit.setText(content)
                self.file_path = file_path
                self.update_file_path_label()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load file: {str(e)}")
    
    def save_file(self):
        """Save the current file"""
        if not self.file_path:
            self.save_file_as()
            return
        
        try:
            with open(self.file_path, 'w', encoding='utf-8') as file:
                content = self.text_edit.toPlainText()
                file.write(content)
            QMessageBox.information(self, "Success", "File saved successfully.")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save file: {str(e)}")
    
    def save_file_as(self):
        """Save the current file with a new name"""
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save File As", "", "XML Files (*.xml *.urdf);;All Files (*)"
        )
        
        if file_path:
            self.file_path = file_path
            self.save_file()
            self.update_file_path_label()
    
    def update_file_path_label(self):
        """Update the file path label"""
        if self.file_path:
            self.file_path_label.setText(self.file_path)
            self.setWindowTitle(f"XML Editor - {os.path.basename(self.file_path)}")
        else:
            self.file_path_label.setText("No file loaded")
            self.setWindowTitle("XML Editor")
    
    def update_model(self):
        """Update the model in the viewer with the current text content"""
        if self.update_callback:
            content = self.text_edit.toPlainText()
            self.update_callback(content)
        else:
            QMessageBox.warning(self, "Warning", "Update callback not set.")
    
    def closeEvent(self, event):
        """Handle window close event"""
        # Accept the close event
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # Get file path from command line arguments
    file_path = sys.argv[1] if len(sys.argv) > 1 else None
    
    editor = XMLEditor(file_path)
    editor.show()
    
    sys.exit(app.exec_())
