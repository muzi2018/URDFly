#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5.QtWidgets import (
    QDialog,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QTextEdit,
    QLabel,
    QSplitter,
    QWidget,
    QGroupBox
)
from PyQt5.QtCore import Qt


class MDHDialog(QDialog):
    """Dialog for displaying MDH parameters and generating code"""
    
    def __init__(self, parent=None, chain_name="", mdh_parameters=None):
        super().__init__(parent)
        self.chain_name = chain_name
        self.mdh_parameters = mdh_parameters if mdh_parameters else []
        self.init_ui()
        
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle(f"MDH Parameters - {self.chain_name}")
        self.setMinimumWidth(1000)
        self.setMinimumHeight(600)
        
        # Main layout
        main_layout = QHBoxLayout(self)
        
        # Create left panel for MDH parameters
        left_panel = self.create_left_panel()
        
        # Create right panel for code display
        right_panel = self.create_right_panel()
        
        # Add panels to main layout with splitter
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setSizes([400, 600])  # Set initial sizes
        
        main_layout.addWidget(splitter)
        
    def create_left_panel(self):
        """Create the left panel with MDH parameters table and buttons"""
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        
        # Add title
        title_label = QLabel("MDH Parameters")
        title_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        left_layout.addWidget(title_label)
        
        # Create table for MDH parameters
        self.mdh_table = QTableWidget()
        self.mdh_table.setColumnCount(5)  # Joint, theta, d, a, alpha
        self.mdh_table.setHorizontalHeaderLabels(["Joint", "θ (rad)", "d", "a", "α (rad)"])
        
        # Set row count
        self.mdh_table.setRowCount(len(self.mdh_parameters))
        
        # Fill table with MDH parameters
        for i, params in enumerate(self.mdh_parameters):
            # Joint name
            joint_item = QTableWidgetItem(f"Joint {i+1}")
            self.mdh_table.setItem(i, 0, joint_item)
            
            # MDH parameters: theta, d, a, alpha
            for j, param in enumerate(params):
                param_item = QTableWidgetItem(f"{param:.4f}")
                self.mdh_table.setItem(i, j+1, param_item)
        
        # Resize columns to content
        self.mdh_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        left_layout.addWidget(self.mdh_table)
        
        # Create buttons
        buttons_layout = QHBoxLayout()
        
        self.btn_forward_kinematics = QPushButton("Forward Kinematics")
        self.btn_forward_kinematics.clicked.connect(self.on_forward_kinematics)
        
        self.btn_jacobian = QPushButton("Jacobian")
        self.btn_jacobian.clicked.connect(self.on_jacobian)
        
        buttons_layout.addWidget(self.btn_forward_kinematics)
        buttons_layout.addWidget(self.btn_jacobian)
        
        left_layout.addLayout(buttons_layout)
        
        return left_widget
    
    def create_right_panel(self):
        """Create the right panel with header and cpp file display areas"""
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        
        # Create splitter for top and bottom sections
        vertical_splitter = QSplitter(Qt.Vertical)
        
        # Top section for .h header file
        header_group = QGroupBox(".h Header File")
        header_layout = QVBoxLayout(header_group)
        self.header_text_edit = QTextEdit()
        self.header_text_edit.setReadOnly(True)
        self.header_text_edit.setPlaceholderText("Header file content will be displayed here...")
        header_layout.addWidget(self.header_text_edit)
        
        # Bottom section for .cpp file
        cpp_group = QGroupBox(".cpp Source File")
        cpp_layout = QVBoxLayout(cpp_group)
        self.cpp_text_edit = QTextEdit()
        self.cpp_text_edit.setReadOnly(True)
        self.cpp_text_edit.setPlaceholderText("Source file content will be displayed here...")
        cpp_layout.addWidget(self.cpp_text_edit)
        
        # Add both sections to vertical splitter
        vertical_splitter.addWidget(header_group)
        vertical_splitter.addWidget(cpp_group)
        vertical_splitter.setSizes([300, 300])  # Equal split
        
        right_layout.addWidget(vertical_splitter)
        
        return right_widget
    
    def on_forward_kinematics(self):
        """Handle Forward Kinematics button click"""
        # TODO: Implement forward kinematics calculation and code generation
        print("Forward Kinematics button clicked")
        # For now, just show placeholder text
        self.header_text_edit.setText("// Forward Kinematics header code will be generated here")
        self.cpp_text_edit.setText("// Forward Kinematics implementation code will be generated here")
    
    def on_jacobian(self):
        """Handle Jacobian button click"""
        # TODO: Implement Jacobian calculation and code generation
        print("Jacobian button clicked")
        # For now, just show placeholder text
        self.header_text_edit.setText("// Jacobian header code will be generated here")
        self.cpp_text_edit.setText("// Jacobian implementation code will be generated here")
    
    def update_mdh_parameters(self, mdh_parameters):
        """Update the MDH parameters in the table"""
        self.mdh_parameters = mdh_parameters
        
        # Update table row count
        self.mdh_table.setRowCount(len(self.mdh_parameters))
        
        # Fill table with new MDH parameters
        for i, params in enumerate(self.mdh_parameters):
            # Joint name
            joint_item = QTableWidgetItem(f"Joint {i+1}")
            self.mdh_table.setItem(i, 0, joint_item)
            
            # MDH parameters: theta, d, a, alpha
            for j, param in enumerate(params):
                param_item = QTableWidgetItem(f"{param:.4f}")
                self.mdh_table.setItem(i, j+1, param_item)
