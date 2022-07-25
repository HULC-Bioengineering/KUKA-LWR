import vtk
import csv
import numpy

_INPUT_FILE = './output.csv'

class PointCloud(object):

  def __init__(self):
    self.__points = vtk.vtkPoints()
    self.__cells = vtk.vtkCellArray()
    self.__color = vtk.vtkDoubleArray() # Color
    self.__poly_data = vtk.vtkPolyData()
    self.__mapper = vtk.vtkPolyDataMapper()
    self.__actor = vtk.vtkActor()

    self.__color.SetName('ColorArray')

    self.__poly_data.SetPoints(self.__points)
    self.__poly_data.SetVerts(self.__cells)
    self.__poly_data.GetPointData().SetScalars(self.__color)
    self.__poly_data.GetPointData().SetActiveScalars('ColorArray')
    
    self.__mapper.SetInput(self.__poly_data)
    self.__mapper.SetColorModeToDefault()
    self.__mapper.SetScalarVisibility(1)

    self.__actor.SetMapper(self.__mapper)

  def add_point(self, point):
    pointId = self.__points.InsertNextPoint(point[:])
    self.__color.InsertNextValue(point[2]) # Z-Value
    self.__cells.InsertNextCell(1) # add cell
    self.__cells.InsertCellPoint(pointId) # index in __points array

    self.__points.Modified()
    self.__cells.Modified()
    self.__color.Modified()

  def get_actor(self): return self.__actor

def main():
  cloud = PointCloud()

  reader = csv.reader(open(_INPUT_FILE, 'r'))
  for row in reader:

    point = [float(row[0]), float(row[1]), float(row[2])]
    cloud.add_point(point)
    
  # Renderer
  renderer = vtk.vtkRenderer()
  renderer.AddActor(cloud.get_actor())
  renderer.SetBackground(1, 1, 1)
  renderer.ResetCamera()

  # Render Window
  renderWindow = vtk.vtkRenderWindow()
  renderWindow.AddRenderer(renderer)
  

  # Interactor
  renderWindowInteractor = vtk.vtkRenderWindowInteractor()
  renderWindowInteractor.SetRenderWindow(renderWindow)

  # Begin Interaction
  renderWindow.Render()
  renderWindowInteractor.Start()
  
if __name__=='__main__':
  main()
