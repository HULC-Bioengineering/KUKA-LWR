from Reamer import Reamer
from Chart import Chart
import argparse
import sys

# get command line arguments input, output, loadcel
parser = argparse.ArgumentParser()
parser.add_argument("-lc", "--loadcell", help="file containing load cell data")
parser.add_argument("-ot", "--opticaltracker", help="file containing optical tracker data")
parser.add_argument("-o", "--output", help="output file")
args = parser.parse_args()

# error check command line arguments
if not args.loadcell:
  parser.error("please include loadcell input file")
if not args.opticaltracker:
  parser.error("please include optical tracker input file")
if not args.output:
  parser.error("pleasae include output file")


############ Setup ######################

# Create objects reamer and chart
r = Reamer()
c = Chart()


# load data from file(s) into reamer
r.loadDataFromFile(args.loadcell, args.opticaltracker)

# setup visitor 
r.accept(c)

############# Data Manipulation ############

c.normalize()
c.autoTrim()
#c.selectRange(1, 10)
c.normalize()
c.biasLoadCell()
c.biasOpticalTracker()
c.toExcel(args.output)
#c.isolate1push(1)

############## Saving #####################

#c.toExcelInterpolated(args.output)
#c.toExcelPushSeperated(args.output)

############# Plotting ####################

#c.plotPoints()
#c.bestFitCurve()
#c.display()
