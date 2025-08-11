import vtk

class URDFModel:
    """Class to represent a URDF link with its mesh and transformation"""

    def __init__(self, name, mesh_file, mesh_transform, link_frame, color=None):
        self.name = name
        self.mesh_file = mesh_file
        self.mesh_transform = mesh_transform  # 4x4 transformation matrix
        self.link_frame = link_frame

        # Create the STL reader
        self.reader = vtk.vtkSTLReader()
        self.reader.SetFileName(mesh_file)
        self.reader.Update()

        # Create mapper and actor
        self.mapper = vtk.vtkPolyDataMapper()
        self.mapper.SetInputConnection(self.reader.GetOutputPort())

        self.actor = vtk.vtkActor()
        self.actor.SetMapper(self.mapper)

        # Set color for the actor
        if color is None:
            self.set_random_color()
        else:
            self.actor.GetProperty().SetColor(color[0], color[1], color[2])
            if len(color) > 3:
                self.actor.GetProperty().SetOpacity(color[3])

        # Store original color for highlighting/unhighlighting
        self.original_color = self.actor.GetProperty().GetColor()

        # Flag to track if model is highlighted
        self.is_highlighted = False

        # Initialize transparency (1.0 = fully opaque, 0.0 = fully transparent)
        self.transparency = 1.0

        # Apply the transformation
        self.apply_transform(self.mesh_transform)

        # Create coordinate axes for this link
        self.axes_actor = self.create_axes_actor(self.link_frame)

    def set_random_color(self):
        """Set a random color for the model"""
        import random

        r = random.random()
        g = random.random()
        b = random.random()
        self.actor.GetProperty().SetColor(r, g, b)
        # Store the original color
        self.original_color = (r, g, b)

    def highlight(self):
        """Highlight the model by changing its appearance"""
        if not self.is_highlighted:
            # Store current color if not already stored
            if not hasattr(self, "original_color"):
                self.original_color = self.actor.GetProperty().GetColor()

            # Set highlight properties
            self.actor.GetProperty().SetColor(1.0, 1.0, 0.0)  # Yellow highlight
            self.actor.GetProperty().SetEdgeVisibility(True)
            self.actor.GetProperty().SetEdgeColor(1.0, 1.0, 1.0)  # White edges
            self.actor.GetProperty().SetLineWidth(2.0)
            self.actor.GetProperty().SetSpecular(0.6)
            self.actor.GetProperty().SetSpecularPower(30)
            self.is_highlighted = True

    def unhighlight(self):
        """Remove highlighting from the model"""
        if self.is_highlighted:
            # Restore original color and properties
            self.actor.GetProperty().SetColor(self.original_color)
            self.actor.GetProperty().SetEdgeVisibility(False)
            self.actor.GetProperty().SetSpecular(0.0)
            self.is_highlighted = False

    def apply_transform(self, transform_matrix):
        """Apply a 4x4 transformation matrix to the actor"""
        # Convert numpy matrix to vtk transform
        vtk_transform = vtk.vtkTransform()
        vtk_transform.SetMatrix(transform_matrix.flatten())

        # Apply the transform to the actor
        self.actor.SetUserTransform(vtk_transform)

    def set_transparency(self, transparency):
        """Set the transparency of the model (1.0 = opaque, 0.0 = transparent)"""
        self.transparency = transparency
        # Apply transparency to the actor
        self.actor.GetProperty().SetOpacity(transparency)

    def create_axes_actor(self, transform_matrix):
        """Create coordinate axes for this link"""
        # Create axes
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(0.05, 0.05, 0.05)  # Set the length of the axes
        axes.SetShaftType(0)
        axes.SetAxisLabels(0)
        axes.SetCylinderRadius(0.02)

        # Create transform
        vtk_transform = vtk.vtkTransform()
        vtk_transform.SetMatrix(transform_matrix.flatten())
        axes.SetUserTransform(vtk_transform)

        return axes