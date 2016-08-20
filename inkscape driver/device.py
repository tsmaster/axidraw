# axidraw.py
# Part of the AxiDraw driver for Inkscape
# https://github.com/evil-mad/AxiDraw
#
# Version 1.1.0, dated August 9, 2016.
# 
# Requires Pyserial 2.7.0 or newer. Pyserial 3.0 recommended.
#
# Copyright 2016 Windell H. Oskay, Evil Mad Scientist Laboratories
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

import math
from math import sqrt
from array import *
import gettext
import serial
import string
import time

import ebb_serial		# https://github.com/evil-mad/plotink
import ebb_motion		# https://github.com/evil-mad/plotink  Requires version 0.5
import plot_utils		# https://github.com/evil-mad/plotink  Requires version 0.4

import axidraw_conf       	#Some settings can be changed here.

F_DEFAULT_SPEED = 25
N_PEN_DOWN_DELAY = 400    # delay (ms) for the pen to go down before the next move
N_PEN_UP_DELAY = 400      # delay (ms) for the pen to up down before the next move

N_PEN_UP_POS = 50      # Default pen-up position
N_PEN_DOWN_POS = 40      # Default pen-down position

N_SERVOSPEED = 150			# Default pen-lift speed 
N_DEFAULT_LAYER = 1			# Default inkscape layer

class WCB:
	def __init__( self ):
		self.start_time = time.time()

		self.serialPort = None
		self.bPenIsUp = None  #Initial state of pen is neither up nor down, but _unknown_.
		self.virtualPenIsUp = False  #Keeps track of pen postion when stepping through plot before resuming
		self.ignoreLimits = False


		fX = None
		fY = None 
		self.fCurrX = axidraw_conf.StartPos_X
		self.fCurrY = axidraw_conf.StartPos_Y 
		self.ptFirst = ( axidraw_conf.StartPos_X, axidraw_conf.StartPos_Y)
		self.bStopped = False
		self.fSpeed = 1
		self.resumeMode = False
		self.nodeCount = int( 0 )		#NOTE: python uses 32-bit ints.
		self.nodeTarget = int( 0 )
		self.pathcount = int( 0 )
		self.LayersFoundToPlot = False
		self.LayerOverrideSpeed = False
		self.LayerOverridePenDownHeight = False
		self.LayerPenDownPosition = -1
		self.LayerPenDownSpeed = -1
                self.optionsPenUpPosition = N_PEN_UP_POS
                self.optionsPenDownPosition = N_PEN_DOWN_POS
                self.optionsPenDownSpeed = F_DEFAULT_SPEED
                self.optionsPenUpDelay = N_PEN_UP_DELAY
                self.optionsServoUpSpeed = N_SERVOSPEED

                self.optionsPenDownSpeed = F_DEFAULT_SPEED
                self.optionsRapidSpeed = F_DEFAULT_SPEED
                self.optionsServoUpSpeed = N_SERVOSPEED
                self.optionsPenUpDelay = N_PEN_UP_DELAY
                self.optionsServoDownSpeed = N_SERVOSPEED
                self.optionsPenDownDelay = N_PEN_DOWN_DELAY
                self.optionsReport_time = False
                self.optionsSlow_slices = False
                self.optionsConstSpeed = False
                self.optionsAutoRotate = False
                self.optionsSmoothness = 2.0
                self.optionsCornering = 2.0
                self.optionsResolution = 2
                self.optionsWalkDistance = 1
                self.optionsLayernumber = N_DEFAULT_LAYER

                

		self.penUpDistance = 0.0
		self.penDownDistance = 0.0
		
		#Values read from file:
		self.svgLayer_Old = int( 0 )
		self.svgNodeCount_Old = int( 0 )
		self.svgDataRead_Old = False
		self.svgLastPath_Old = int( 0 )
		self.svgLastPathNC_Old = int( 0 )
                print "initializing last known pos to 0"
		self.svgLastKnownPosX_Old = float( 0.0 )
		self.svgLastKnownPosY_Old = float( 0.0 )
		self.svgPausedPosX_Old = float( 0.0 )
		self.svgPausedPosY_Old = float( 0.0 )	
		
		#New values to write to file:
		self.svgLayer = int( 0 )
		self.svgNodeCount = int( 0 )
		self.svgDataRead = False
		self.svgLastPath = int( 0 )
		self.svgLastPathNC = int( 0 )
		self.svgLastKnownPosX = float( 0.0 )
		self.svgLastKnownPosY = float( 0.0 )
		self.svgPausedPosX = float( 0.0 )
		self.svgPausedPosY = float( 0.0 )	

		self.backlashStepsX = int(0)
		self.backlashStepsY = int(0)	 
		self.XBacklashFlag = True
		self.YBacklashFlag = True
		
		self.manConfMode = False
		self.PrintFromLayersTab = False

		self.svgWidth = 0 
		self.svgHeight = 0
		self.printPortrait = False
		
		self.xBoundsMax = axidraw_conf.N_PAGE_WIDTH
		self.xBoundsMin = axidraw_conf.StartPos_X
		self.yBoundsMax = axidraw_conf.N_PAGE_HEIGHT
		self.yBoundsMin = axidraw_conf.StartPos_Y
		
		self.svgTransform = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
		
		self.stepsPerInch = 0 # must be set to a nonzero value before plotting.
		self.PenDownSpeed = 0.25 * axidraw_conf.Speed_Scale #Default speed when pen is down		
		self.PenUpSpeed = 0.75 * axidraw_conf.Speed_Scale #Default speed when pen is down	

		# So that we only generate a warning once for each
		# unsupported SVG element, we use a dictionary to track
		# which elements have received a warning
		self.warnings = {}
		self.warnOutOfBounds = False


					 
	def setupCommand( self, doToggle ):
		"""Execute commands from the "setup" tab"""

		if self.serialPort is None:
			return

		self.ServoSetupWrapper()

                if not doToggle:
			self.penUp()
			ebb_motion.sendDisableMotors(self.serialPort)	
                else:
			ebb_motion.TogglePen(self.serialPort)

        def disable(self):
		self.penUp()
		ebb_motion.sendDisableMotors(self.serialPort)	
                

	def manualCommand( self ):
		"""Execute commands from the "manual" tab"""

		if self.serialPort is None:
			return 

		if self.options.manualType == "raise-pen":
			self.ServoSetupWrapper()
			self.penUp()

		elif self.options.manualType == "lower-pen":
			self.ServoSetupWrapper()
			self.penDown()

		elif self.options.manualType == "enable-motors":
			self.EnableMotors()

		elif self.options.manualType == "disable-motors":
			ebb_motion.sendDisableMotors(self.serialPort)	

		elif self.options.manualType == "version-check":
			strVersion = ebb_serial.query( self.serialPort, 'V\r' )
			print( 'I asked the EBB for its version info, and it replied:\n ' + strVersion )

		else:  # self.options.manualType is walk motor:
			if self.options.manualType == "walk-y-motor":
				nDeltaX = 0
				nDeltaY = self.options.WalkDistance
			elif self.options.manualType == "walk-x-motor":
				nDeltaY = 0
				nDeltaX = self.options.WalkDistance
			else:
				return
			
			self.fSpeed = self.PenDownSpeed
				
 			self.EnableMotors() #Set plotting resolution 
			self.fCurrX = self.svgLastKnownPosX_Old + axidraw_conf.StartPos_X
			self.fCurrY = self.svgLastKnownPosY_Old + axidraw_conf.StartPos_Y
			self.ignoreLimits = True
			fX = self.fCurrX + nDeltaX   #Note: Walking motors is STRICTLY RELATIVE TO INITIAL POSITION.
			fY = self.fCurrY + nDeltaY
			self.plotSegmentWithVelocity( fX, fY, 0, 0)


        def moveRel(self, rx, ry, ignoreLimits):
		self.fSpeed = self.PenDownSpeed
				
 		self.EnableMotors() #Set plotting resolution 
		self.fCurrX = self.svgLastKnownPosX + axidraw_conf.StartPos_X
		self.fCurrY = self.svgLastKnownPosY + axidraw_conf.StartPos_Y
                print "last known pos at beginning of moveRel:", self.svgLastKnownPosX, self.svgLastKnownPosY
                print "curr pos at beginning of moveRel:", self.fCurrX, self.fCurrY
		self.ignoreLimits = ignoreLimits
		fX = self.fCurrX + rx   #Note: Walking motors is STRICTLY RELATIVE TO INITIAL POSITION.
		fY = self.fCurrY + ry
                print "move destination:", fX, fY
		self.plotSegmentWithVelocity( fX, fY, 0, 0)
                print "after move pos:", self.fCurrX, self.fCurrY

        def moveAbs(self, ax, ay, ignoreLimits):
		self.fSpeed = self.PenDownSpeed
				
 		self.EnableMotors() #Set plotting resolution 
		self.fCurrX = self.svgLastKnownPosX + axidraw_conf.StartPos_X
		self.fCurrY = self.svgLastKnownPosY + axidraw_conf.StartPos_Y
                print "last known pos at beginning of moveAbs:", self.svgLastKnownPosX, self.svgLastKnownPosY
                print "curr pos at beginning of moveAbs:", self.fCurrX, self.fCurrY
		self.ignoreLimits = ignoreLimits
                print "move destination:", ax, ay
		self.plotSegmentWithVelocity( ax, ay, 0, 0)
                print "after move pos:", self.fCurrX, self.fCurrY
                
                

	def plotPath( self, path, matTransform ):
		'''
		Plot the path while applying the transformation defined
		by the matrix [matTransform].
		'''
		# turn this path into a cubicsuperpath (list of beziers)...

		d = path.get( 'd' )
		if len( simplepath.parsePath( d ) ) == 0:
			return

		if self.plotCurrentLayer:
			p = cubicsuperpath.parsePath( d )

			# ...and apply the transformation to each point
			applyTransformToPath( matTransform, p )
	
			# p is now a list of lists of cubic beziers [control pt1, control pt2, endpoint]
			# where the start-point is the last point in the previous segment.
			for sp in p:
			
				plot_utils.subdivideCubicPath( sp, 0.02 / self.optionsSmoothness )
				nIndex = 0

				singlePath = []		
				if self.plotCurrentLayer:
					for csp in sp:
						if self.bStopped:
							return
						if (self.printPortrait):
							fX = float( csp[1][1] ) #Flipped X/Y
							fY = ( self.svgWidth) - float( csp[1][0] )
						else:
							fX = float( csp[1][0] ) # Set move destination
							fY = float( csp[1][1] )

						if nIndex == 0:
							if (plot_utils.distance(fX - self.fCurrX,fY - self.fCurrY) > axidraw_conf.MIN_GAP):
								self.penUp()
								self.plotSegmentWithVelocity( fX, fY, 0, 0)
						elif nIndex == 1:
							self.penDown() 
						nIndex += 1

						singlePath.append([fX,fY])
	
					self.PlanTrajectory(singlePath)
	
			if ( not self.bStopped ):	#an "index" for resuming plots quickly-- record last complete path
				self.svgLastPath = self.pathcount #The number of the last path completed
				self.svgLastPathNC = self.nodeCount #the node count after the last path was completed.			


	def PlanTrajectory( self, inputPath ):
		'''
		Plan the trajectory for a full path, accounting for linear acceleration.
		Inputs: Ordered (x,y) pairs to cover.
		Output: A list of segments to plot, of the form (Xfinal, Yfinal, Vinitial, Vfinal)

		Note: Native motor axes are Motor 1, Motor 2.
			Motor1Steps = xSteps + ySteps
			Motor2Steps = xSteps - ysteps
			
		Important note: This routine uses *inch* units (inches, inches/second, etc.). 
		
		'''
		
# 		spewTrajectoryDebugData = True
		spewTrajectoryDebugData = False
		
		if spewTrajectoryDebugData:
			print( '\nPlanTrajectory()\n')

		if self.bStopped:
			return
		if ( self.fCurrX is None ):
			return

		#check page size limits:
		if (self.ignoreLimits == False):
			for xy in inputPath:
				xy[0], xBounded = plot_utils.checkLimits( xy[0], self.xBoundsMin, self.xBoundsMax )
				xy[1], yBounded = plot_utils.checkLimits( xy[1], self.yBoundsMin, self.yBoundsMax )
				if (xBounded or yBounded):
					self.warnOutOfBounds = True
							
		#Handle simple segments (lines) that do not require any complex planning:
		if (len(inputPath) < 3):
			if spewTrajectoryDebugData:
				print( 'SHORTPATH ESCAPE: ')	
			self.plotSegmentWithVelocity( xy[0], xy[1], 0, 0)							  
			return
			
		#For other trajectories, we need to go deeper.
		TrajLength = len(inputPath)

		if spewTrajectoryDebugData:
			for xy in inputPath:
				print( 'x: %1.2f,  y: %1.2f' %(xy[0],xy[1]))
			print( '\nTrajLength: '+str(TrajLength) + '\n')

		#Absolute maximum and minimum speeds allowed: 

		#Values such as PenUpSpeed are in units of _steps per second_.  
		# However, to simplify our kinematic calculations, 
		# we now presently switch into inches per second. 

		# Maximum travel speed
		if ( self.virtualPenIsUp ):	
			speedLimit = self.PenUpSpeed  / self.stepsPerInch
		else:		
			speedLimit = self.PenDownSpeed  / self.stepsPerInch

		TrajDists = array('f')	 #float, Segment length (distance) when arriving at the junction
		TrajVels = array('f')	 #float, Velocity when arriving at the junction
		TrajVectors = []		#Array that will hold normalized unit vectors along each segment

		TrajDists.append(0.0)	#First value, at time t = 0
		TrajVels.append(0.0)	#First value, at time t = 0

		for i in xrange(1, TrajLength):
			#Distance per segment:
			tmpDist = plot_utils.distance( inputPath[i][0] - inputPath[i - 1][0] ,
			inputPath[i][1] - inputPath[i - 1][1] )
			TrajDists.append(tmpDist)
			#Normalized unit vectors:
			
			if (tmpDist == 0):
				tmpDist = 1
			tmpX = (inputPath[i][0] - inputPath[i - 1][0]) / tmpDist
			tmpY = (inputPath[i][1] - inputPath[i - 1][1]) / tmpDist
			TrajVectors.append([tmpX,tmpY])

		if spewTrajectoryDebugData:
			for dist in TrajDists:
				print( 'TrajDists: %1.3f' % dist )
			print( '\n')

		#time to reach full speed (from zero), at maximum acceleration. Defined in settings:

		if ( self.virtualPenIsUp ):	
			tMax = axidraw_conf.ACCEL_TIME_PU			
		else:		
			tMax = axidraw_conf.ACCEL_TIME			

		# acceleration/deceleration rate: (Maximum speed) / (time to reach that speed)
		accelRate = speedLimit / tMax
		
		#Distance that is required to reach full speed, from zero speed:  (1/2) a t^2
		accelDist = 0.5 * accelRate * tMax  * tMax

		if spewTrajectoryDebugData:		
			print( 'speedLimit: %1.3f' % speedLimit )
			print( 'tMax: %1.3f' % tMax )
			print( 'accelRate: %1.3f' % accelRate )
			print( 'accelDist: %1.3f' % accelDist )
			CosinePrintArray = array('f')
			
			
		'''
		Now, step through every vertex in the trajectory, and calculate what the speed
		should be when arriving at that vertex.
		
		In order to do so, we need to understand how the trajectory will evolve in terms 
		of position and velocity for a certain amount of time in the future, past that vertex. 
		The most extreme cases of this is when we are traveling at 
		full speed initially, and must come to a complete stop.
			(This is actually more sudden than if we must reverse course-- that must also
			go through zero velocity at the same rate of deceleration, and a full reversal
			that does not occur at the path end might be able to have a 
			nonzero velocity at the endpoint.)
			
		Thus, we look ahead from each vertex until one of the following occurs:
			(1) We have looked ahead by at least tMax, or
			(2) We reach the end of the path.

		The data that we have to start out with is this:
			- The position and velocity at the previous vertex
			- The position at the current vertex
			- The position at subsequent vertices
			- The velocity at the final vertex (zero)

		To determine the correct velocity at each vertex, we will apply the following rules:
		
		(A) For the first point, V(i = 0) = 0.

		(B) For the last point point, Vi = 0 as well.
		
		(C) If the length of the segment is greater than the distance 
		required to reach full speed, then the vertex velocity may be as 
		high as the maximum speed.
		
		(D) However, if the length of the segment is less than the total distance
		required to get to full speed, then the velocity at that vertex
		is limited by to the value that can be reached from the initial
		starting velocity, in the distance given.
				
		(E) The maximum velocity through the junction is also limited by the
		turn itself-- if continuing straight, then we do not need to slow down
		as much as if we were fully reversing course. 
		We will model each corner as a short curve that we can accelerate around.
		
		(F) To calculate the velocity through each turn, we must _look ahead_ to
		the subsequent (i+1) vertex, and determine what velocity 
		is appropriate when we arrive at the next point. 
		
		Because future points may be close together-- the subsequent vertex could
		occur just before the path end -- we actually must look ahead past the 
		subsequent (i + 1) vertex, all the way up to the limits that we have described 
		(e.g., tMax) to understand the subsequent behavior. Once we have that effective
		endpoint, we can work backwards, ensuring that we will be able to get to the
		final speed/position that we require. 
		
		A less complete (but far simpler) procedure is to first complete the trajectory
		description, and then -- only once the trajectory is complete -- go back through,
		but backwards, and ensure that we can actually decelerate to each velocity.

		(G) The minimum velocity through a junction may be set to a constant.
		There is often some (very slow) speed -- perhaps a few percent of the maximum speed
		at which there are little or no resonances. Even when the path must directly reverse
		itself, we can usually travel at a non-zero speed. This, of course, presumes that we 
		still have a solution for getting to the endpoint at zero speed.
		'''

		delta = self.optionsCornering / 1000  #Corner rounding/tolerance factor-- not sure how high this should be set.
		
		for i in xrange(1, TrajLength - 1):
			Dcurrent = TrajDists[i]		# Length of the segment leading up to this vertex
			VPrevExit = TrajVels[i-1]	# Velocity when leaving previous vertex

			'''
			Velocity at vertex: Part I
			
			Check to see what our plausible maximum speeds are, from 
			acceleration only, without concern about cornering, nor deceleration.
			'''

			if (Dcurrent > accelDist):		
				#There _is_ enough distance in the segment for us to either
				# accelerate to maximum speed or come to a full stop before this vertex.
				VcurrentMax = speedLimit
				if spewTrajectoryDebugData:
					print( 'Speed Limit on vel : '+str(i))
			else:
				#There is _not necessarily_ enough distance in the segment for us to either
				# accelerate to maximum speed or come to a full stop before this vertex.
				# Calculate how much we *can* swing the velocity by:	
				
				VcurrentMax = plot_utils.vFinal_Vi_A_Dx(VPrevExit,accelRate, Dcurrent)
				if (VcurrentMax > speedLimit):
					VcurrentMax = speedLimit
					
				if spewTrajectoryDebugData:
					print( 'TrajVels I: %1.3f' % VcurrentMax )
	
			'''
			Velocity at vertex: Part II 
			
			Assuming that we have the same velocity when we enter and
			leave a corner, our acceleration limit provides a velocity
			that depends upon the angle between input and output directions.
			
			The cornering algorithm models the corner as a slightly smoothed corner,
			to estimate the angular acceleration that we encounter:
			https://onehossshay.wordpress.com/2011/09/24/improving_grbl_cornering_algorithm/
			
			The dot product of the unit vectors is equal to the cosine of the angle between the
			two unit vectors, giving the deflection between the incoming and outgoing angles. 
			Note that this angle is (pi - theta), in the convention of that article, giving us
			a sign inversion. [cos(pi - theta) = - cos(theta)]
			'''
			
			cosineFactor = - plot_utils.dotProductXY(TrajVectors[i - 1],TrajVectors[i]) 

			if spewTrajectoryDebugData:
				CosinePrintArray.append(cosineFactor) 

			rootFactor = sqrt((1 - cosineFactor)/2)
			denominator =  1 - rootFactor
			if (denominator > 0.0001):
				Rfactor = (delta * rootFactor) / denominator
			else:	
				Rfactor = 100000
			VjunctionMax = sqrt(accelRate * Rfactor)
			
			if (VcurrentMax > VjunctionMax):
				VcurrentMax = VjunctionMax
				
			TrajVels.append( VcurrentMax)	# "Forward-going" speed limit for velocity at this particular vertex.
		TrajVels.append( 0.0 	)				# Add zero velocity, for final vertex.

		if spewTrajectoryDebugData:
			print( ' ')
			for dist in CosinePrintArray:
				print( 'Cosine Factor: %1.3f' % dist )
			print( ' ')
			
			for dist in TrajVels:
				print( 'TrajVels II: %1.3f' % dist )
			print( ' ')	

		'''			
		Velocity at vertex: Part III

		We have, thus far, ensured that we could reach the desired velocities, going forward, but
		have also assumed an effectively infinite deceleration rate.		

		We now go through the completed array in reverse, limiting velocities to ensure that we 
		can properly decelerate in the given distances.		
		'''
		
		for j in xrange(1, TrajLength):
			i = TrajLength - j	# Range: From (TrajLength - 1) down to 1.

			Vfinal = TrajVels[i]
			Vinitial = TrajVels[i - 1]
			SegLength = TrajDists[i]



			if (Vinitial > Vfinal) and (SegLength > 0): 	
				VInitMax = plot_utils.vInitial_VF_A_Dx(Vfinal,-accelRate,SegLength)

				if spewTrajectoryDebugData:
					print( 'VInit Calc: (Vfinal = %1.3f, accelRate = %1.3f, SegLength = %1.3f) ' 
					% (Vfinal, accelRate, SegLength))

				if (VInitMax < Vinitial):
					Vinitial = VInitMax 
				TrajVels[i - 1] = Vinitial
				
		if spewTrajectoryDebugData:
			for dist in TrajVels:
				print( 'TrajVels III: %1.3f' % dist )

			print( ' ')

		for i in xrange(1, TrajLength):			
			self.plotSegmentWithVelocity( inputPath[i][0] , inputPath[i][1] ,TrajVels[i-1] , TrajVels[i])

	def plotSegmentWithVelocity( self, xDest, yDest, Vi, Vf  ):
		''' 
		Control the serial port to command the machine to draw
		a straight line segment, with basic acceleration support. 
		
		Inputs: 	Destination (x,y)
					Initial velocity
					Final velocity
		
		Method: Divide the segment up into smaller segments, each
		of which has constant velocity. 
		Send commands out the com port as a set of short line segments
		(dx, dy) with specified durations (in ms) of how long each segment
		takes to draw.the segments take to draw. 
		Uses linear ("trapezoid") acceleration and deceleration strategy.
		
		Inputs are expected be in units of inches (for distance) 
			or inches per second (for velocity).
		
		'''	

#		spewSegmentDebugData = False
 		spewSegmentDebugData = True

		if spewSegmentDebugData:
			print( '\nPlotSegment (x = %1.2f, y = %1.2f, Vi = %1.2f, Vf = %1.2f ) ' 
			% (xDest, yDest, Vi, Vf))
			if self.resumeMode:	
				print( 'resumeMode is active')

		ConstantVelMode = False
		if (self.optionsConstSpeed and not self.virtualPenIsUp):
			ConstantVelMode = True

		if self.bStopped:
                        print "we are stopped"
			return
		if ( self.fCurrX is None ):
                        print "no current x"
			return

		#check page size limits:
		if (self.ignoreLimits == False):
			xDest, xBounded = plot_utils.checkLimits( xDest, self.xBoundsMin, self.xBoundsMax )
			yDest, yBounded = plot_utils.checkLimits( yDest, self.yBoundsMin, self.yBoundsMax )
			if (xBounded or yBounded):
                                print "Bounded"
				self.warnOutOfBounds = True

                print "steps per inch:", self.stepsPerInch
                print "dest:",xDest, yDest
                print "curr", self.fCurrX, self.fCurrY
                
		# Distances to move, in motor-step units
		xMovementIdeal = self.stepsPerInch * ( xDest - self.fCurrX )	
		yMovementIdeal = self.stepsPerInch * ( yDest - self.fCurrY )
                print "xMovementIdeal:", xMovementIdeal
                print "yMovementIdeal:", yMovementIdeal

		# Velocity inputs, in motor-step units
		initialVel =  Vi * self.stepsPerInch		#Translate from "inches per second"
		finalVel = Vf * self.stepsPerInch		#Translate from "inches per second"

		# Look at distance to move along 45-degree axes, for native motor steps:
		motorSteps1 = int (round(xMovementIdeal + yMovementIdeal)) # Number of native motor steps required, Axis 1
		motorSteps2 = int (round(xMovementIdeal - yMovementIdeal)) # Number of native motor steps required, Axis 2

                print "motorSteps 1:", motorSteps1
                print "motorSteps 2:", motorSteps2

		plotDistance = plot_utils.distance( motorSteps1, motorSteps2 )
		if (plotDistance < 1.0): #if total movement is less than one step, skip this movement.
			return

		if (self.optionsReport_time): #Also keep track of distance:
			if (self.virtualPenIsUp):
				self.penUpDistance = self.penUpDistance + plotDistance
			else:
				self.penDownDistance = self.penDownDistance +plotDistance

		# Maximum travel speeds:
		# & acceleration/deceleration rate: (Maximum speed) / (time to reach that speed)

		if ( self.virtualPenIsUp ):	
			speedLimit = self.PenUpSpeed
			accelRate = speedLimit / axidraw_conf.ACCEL_TIME_PU	
			
			if plotDistance < (self.stepsPerInch * axidraw_conf.SHORT_THRESHOLD):
				accelRate = speedLimit / axidraw_conf.ACCEL_TIME	
				speedLimit = self.PenDownSpeed
		else:		
			speedLimit = self.PenDownSpeed
			accelRate = speedLimit / axidraw_conf.ACCEL_TIME	
			
		if (initialVel > speedLimit):
			initialVel = speedLimit
		if (finalVel > speedLimit):
			finalVel = speedLimit

		#Times to reach maximum speed, from our initial velocity 
		# vMax = vi + a*t  =>  t = (vMax - vi)/a
		# vf = vMax - a*t   =>  t = -(vf - vMax)/a = (vMax - vf)/a
		# -- These are _maximum_ values. We often do not have enough time/space to reach full speed.

		tAccelMax = (speedLimit - initialVel) / accelRate
		tDecelMax = (speedLimit - finalVel) / accelRate	

		if spewSegmentDebugData:
			print( 'accelRate: ' + str(accelRate) )
			print( 'speedLimit: ' + str(speedLimit) )
			print( 'initialVel: ' + str(initialVel) )
			print( 'finalVel: ' + str(finalVel) )
			print( 'tAccelMax: ' + str(tAccelMax) )
			print( 'tDecelMax: ' + str(tDecelMax) )

	
		#Distance that is required to reach full speed, from our start at speed initialVel:
		# distance = vi * t + (1/2) a t^2
		accelDistMax = ( initialVel * tAccelMax ) + ( 0.5 * accelRate * tAccelMax * tAccelMax )
		# Use the same model for deceleration distance; modeling it with backwards motion:
		decelDistMax = ( finalVel * tDecelMax ) + ( 0.5 * accelRate * tDecelMax * tDecelMax )

		#time slices: Slice travel into intervals that are (say) 30 ms long.
		if (self.optionsSlow_slices):
			timeSlice = axidraw_conf.TIME_SLICE_SLOW #Extra-slow time slices to alleviate communications issues.
		else:
			timeSlice = axidraw_conf.TIME_SLICE	#Default slice intervals

		self.nodeCount += 1		# This whole segment move counts as ONE pause/resume node in our plot
		
		if self.resumeMode:
			if ( self.nodeCount >= (self.nodeTarget)):
				self.resumeMode = False
				if ( not self.virtualPenIsUp ):
					self.penDown()	

		# Declare arrays:
		# These are _normally_ 4-byte integers, but could (theoretically) be 2-byte integers on some systems.
		#   if so, this could cause errors in rare cases (very large/long moves, etc.). 
		# Set up an alert system, just in case!

		durationArray = array('I') # unsigned integer for duration -- up to 65 seconds for a move if only 2 bytes.
		distArray = array('f')	#float
		destArray1 = array('i')	#signed integer
		destArray2 = array('i')	#signed integer

		timeElapsed = 0.0		
		position = 0.0
		velocity = initialVel
		
		'''
		
		Next, we wish to estimate total time duration of this segment. 
		In doing so, we must consider the possible cases:

		Case 1: 'Trapezoid'
			Segment length is long enough to reach full speed.
			Segment length > accelDistMax + decelDistMax
			We will get to full speed, with an opportunity to "coast" at full speed
			in the middle.
			
		Case 2: 'Linear velocity ramp'
			For small enough moves -- say less than 10 intervals (typ 500 ms),
			we do not have significant time to ramp the speed up and down.
			Instead, perform only a simple speed ramp between initial and final.
			
		Case 3: 'Triangle'
			Segment length is not long enough to reach full speed.
			Accelerate from initial velocity to a local maximum speed,
			then decelerate from that point to the final velocity.

		Case 4: 'Constant velocity'
			Use a single, constant velocity for all pen-down movements.
			Also a fallback position, when moves are too short for linear ramps.
			
		In each case, we ultimately construct the trajectory in segments at constant velocity.
		In cases 1-3, that set of segments approximates a linear slope in velocity. 
		
		Because we may end up with slight over/undershoot in position along the paths
		with this approach, we perform a final scaling operation (to the correct distance) at the end.
		
		'''

		if (ConstantVelMode == False) or ( self.virtualPenIsUp ):	#Allow accel when pen is up.		
			if (plotDistance > (accelDistMax + decelDistMax + timeSlice * speedLimit)):
				''' 
				#Case 1: 'Trapezoid'
				'''
			
				if spewSegmentDebugData:
					print( 'Type 1: Trapezoid'+ '\n')	
				speedMax = speedLimit	# We will reach _full cruising speed_!
			
				intervals = int(math.floor(tAccelMax / timeSlice))	# Number of intervals during acceleration
				
				#If intervals == 0, then we are already at (or nearly at) full speed.
				if (intervals > 0):			
					timePerInterval = tAccelMax / intervals			
	
					velocityStepSize = (speedMax - initialVel)/(intervals + 1.0)	
					# For six time intervals of acceleration, first interval is at velocity (max/7)
					# 6th (last) time interval is at 6*max/7
					# after this interval, we are at full speed.
					
					for index in range(0, intervals):		#Calculate acceleration phase
						velocity += velocityStepSize
						timeElapsed += timePerInterval
						position += velocity * timePerInterval
						durationArray.append(int(round(timeElapsed * 1000.0)))
						distArray.append(position)		#Estimated distance along direction of travel
					if spewSegmentDebugData:
						print( 'Accel intervals: '+str(intervals))
							
				#Add a center "coasting" speed interval IF there is time for it.
				coastingDistance = plotDistance - (accelDistMax + decelDistMax)	
								
				if (coastingDistance > (timeSlice * speedMax)):
					# There is enough time for (at least) one interval at full cruising speed.
					velocity = speedMax
					cruisingTime = coastingDistance / velocity
					timeElapsed += cruisingTime
					durationArray.append(int(round(timeElapsed * 1000.0)))
					position += velocity * cruisingTime
					distArray.append(position)		#Estimated distance along direction of travel				
					if spewSegmentDebugData:
						print( 'Coast Distance: '+str(coastingDistance))


	
				intervals = int(math.floor(tDecelMax / timeSlice))	# Number of intervals during deceleration
				
				if (intervals > 0):	
					timePerInterval = tDecelMax / intervals			
					velocityStepSize = (speedMax - finalVel)/(intervals + 1.0)	
	
					for index in range(0, intervals):		#Calculate deceleration phase
						velocity -= velocityStepSize
						timeElapsed += timePerInterval
						position += velocity * timePerInterval
						durationArray.append(int(round(timeElapsed * 1000.0)))
						distArray.append(position)		#Estimated distance along direction of travel
					if spewSegmentDebugData:
						print( 'Decel intervals: '+str(intervals))

			else:
				''' 
				#Case 3: 'Triangle' 
				
				We will _not_ reach full cruising speed, but let's go as fast as we can!
				
				We begin with given: initial velocity, final velocity,
					maximum acceleration rate, distance to travel.
				
				The optimal solution is to accelerate at the maximum rate, to some maximum velocity Vmax,
				and then to decelerate at same maximum rate, to the final velocity. 
				This forms a triangle on the plot of V(t). 
				
				The value of Vmax -- and the time at which we reach it -- may be varied in order to
				accommodate our choice of distance-traveled and velocity requirements.
				(This does assume that the segment requested is self consistent, and planned 
				with respect to our acceleration requirements.)
				
				In a more detail, with short notation Vi = initialVel, Vf = finalVel, 
					Amax = accelRate, Dv = (Vf - Vi)
				
				(i) We accelerate from Vi, at Amax to some maximum velocity Vmax.
				This takes place during an interval of time Ta. 
				
				(ii) We then decelerate from Vmax, to Vf, at the same maximum rate, Amax.
				This takes place during an interval of time Td. 					
				
				(iii) The total time elapsed is Ta + Td
				
				(iv) v = v0 + a * t
					=>	Vmax = Vi + Amax * Ta
					and	Vmax = Vf + Amax * Td    (i.e., Vmax - Amax * Td = Vf)
				
					Thus Td = Ta - (Vf - Vi) / Amax, or    Td = Ta - (Dv / Amax)
					
				(v) The distance covered during the acceleration interval Ta is given by:
					Xa = Vi * Ta + (1/2) Amax * Ta^2
					
					The distance covered during the deceleration interval Td is given by:
					Xd = Vf * Td + (1/2) Amax * Td^2
					
					Thus, the total distance covered during interval Ta + Td is given by:
					plotDistance = Xa + Xd = Vi * Ta + (1/2) Amax * Ta^2 + Vf * Td + (1/2) Amax * Td^2

				(vi) Now substituting in Td = Ta - (Dv / Amax), we find:
					Amax * Ta^2 + 2 * Vi * Ta + ( Vi^2 - Vf^2 )/( 2 * Amax ) - plotDistance = 0
					
					Solving this quadratic equation for Ta, we find:
					Ta = ( sqrt(2 * Vi^2 + 2 * Vf^2 + 4 * Amax * plotDistance) - 2 * Vi ) / ( 2 * Amax )
					
					[We pick the positive root in the quadratic formula, since Ta must be positive.]
				
				(vii) From Ta and part (iv) above, we can find Vmax and Td.
				 
				'''
				
				if spewSegmentDebugData:	
					print( '\nType 3: Triangle' )	
				Ta = ( sqrt(2 * initialVel * initialVel + 2 * finalVel * finalVel + 4 * accelRate * plotDistance) 
					- 2 * initialVel ) / ( 2 * accelRate )
					
				if (Ta < 0) :
					Ta = 0
					if spewSegmentDebugData:	
						print( 'Warning: Negative transit time computed.') #Should not happen. :)

				Vmax = initialVel + accelRate * Ta
				if spewSegmentDebugData:	
					print( 'Vmax: '+str(Vmax))

				intervals = int(math.floor(Ta / timeSlice))	# Number of intervals during acceleration

				if (intervals == 0):
					Ta = 0
				Td = Ta - (finalVel - initialVel) / accelRate
				Dintervals = int(math.floor(Td / timeSlice))	# Number of intervals during acceleration

				if ((intervals + Dintervals) > 4):
					if (intervals > 0):
						if spewSegmentDebugData:	
							print( 'Triangle intervals UP: '+str(intervals))
	
						timePerInterval = Ta / intervals			
						velocityStepSize = (Vmax - initialVel)/(intervals + 1.0)	
						# For six time intervals of acceleration, first interval is at velocity (max/7)
						# 6th (last) time interval is at 6*max/7
						# after this interval, we are at full speed.
						
						for index in range(0, intervals):		#Calculate acceleration phase
							velocity += velocityStepSize
							timeElapsed += timePerInterval
							position += velocity * timePerInterval
							durationArray.append(int(round(timeElapsed * 1000.0)))
							distArray.append(position)		#Estimated distance along direction of travel				
					else:
						if spewSegmentDebugData:	
							print( 'Note: Skipping accel phase in triangle.')
							
	
					if (Dintervals > 0):
						if spewSegmentDebugData:	
							print( 'Triangle intervals Down: '+str(intervals))
		
						timePerInterval = Td / Dintervals			
						velocityStepSize = (Vmax - finalVel)/(Dintervals + 1.0)	
						# For six time intervals of acceleration, first interval is at velocity (max/7)
						# 6th (last) time interval is at 6*max/7
						# after this interval, we are at full speed.
						
						for index in range(0, Dintervals):		#Calculate acceleration phase
							velocity -= velocityStepSize
							timeElapsed += timePerInterval
							position += velocity * timePerInterval
							durationArray.append(int(round(timeElapsed * 1000.0)))
							distArray.append(position)		#Estimated distance along direction of travel				
					else:
						if spewSegmentDebugData:
							print( 'Note: Skipping decel phase in triangle.')
				else:	
					''' 
					#Case 2: 'Linear or constant velocity changes' 
					
					Picked for segments that are shorter than 6 time slices. 
					Linear velocity interpolation between two endpoints.
					
					Because these are typically short segments (not enough time for a good "triangle"--
					we slightly boost the starting speed, by taking its average with Vmax for the segment.
					
					For very short segments (less than 2 time slices), use a single 
						segment with constant velocity.
					'''
					
					if spewSegmentDebugData:								
						print( 'Type 2: Linear'+ '\n')	
					# xFinal = vi * t  + (1/2) a * t^2, and vFinal = vi + a * t 
					# Combining these (with same t) gives: 2 a x = (vf^2 - vi^2)  => a = (vf^2 - vi^2)/2x
					# So long as this 'a' is less than accelRate, we can linearly interpolate in velocity.

					initialVel = ( Vmax + initialVel) / 2  	#Boost initial speed for this segment
					velocity = initialVel					#Boost initial speed for this segment

					localAccel = (finalVel * finalVel - initialVel * initialVel)/ (2.0 * plotDistance)
					
					if (localAccel > accelRate):
						localAccel = accelRate
					elif (localAccel < -accelRate):
						localAccel = -accelRate
					if (localAccel == 0):
						#Initial velocity = final velocity -> Skip to constant velocity routine.
						ConstantVelMode = True
					else:	
						tSegment = (finalVel - initialVel) / localAccel		
							
					intervals = int(math.floor(tSegment / timeSlice))	# Number of intervals during deceleration
					if (intervals > 1):
						timePerInterval = tSegment / intervals			
						velocityStepSize = (finalVel - initialVel)/(intervals + 1.0)										
						# For six time intervals of acceleration, first interval is at velocity (max/7)
						# 6th (last) time interval is at 6*max/7
						# after this interval, we are at full speed.
						
						for index in range(0, intervals):		#Calculate acceleration phase
							velocity += velocityStepSize
							timeElapsed += timePerInterval
							position += velocity * timePerInterval
							durationArray.append(int(round(timeElapsed * 1000.0)))
							distArray.append(position)		#Estimated distance along direction of travel				
					else:
						#Short segment; Not enough time for multiple segments at different velocities. 
						initialVel = Vmax #These are _slow_ segments-- use fastest possible interpretation.
						ConstantVelMode = True

		if (ConstantVelMode):
			'''
			#Case 4: 'Constant Velocity mode'
			'''
			if spewSegmentDebugData:	
				print( '-> [Constant Velocity Mode Segment]'+ '\n')	
			#Single segment with constant velocity.
			
			if (self.optionsConstSpeed and not self.virtualPenIsUp):
				velocity = self.PenDownSpeed 	#Constant pen-down speed		
			elif (finalVel > initialVel):
				velocity = finalVel
			elif (initialVel > finalVel):
				velocity = initialVel	
			elif (initialVel > 0):	#Allow case of two are equal, but nonzero	
				velocity = initialVel	
			else: #Both endpoints are equal to zero.	
				velocity = self.PenDownSpeed /10

			if spewSegmentDebugData:	
				print( 'velocity: '+str(velocity))
					
			timeElapsed = plotDistance / velocity
			durationArray.append(int(round(timeElapsed * 1000.0)))
			distArray.append(plotDistance)		#Estimated distance along direction of travel
			position += plotDistance
			
		''' 
		The time & distance motion arrays for this path segment are now computed.
		Next: We scale to the correct intended travel distance, 
		round into integer motor steps and manage the process
		of sending the output commands to the motors.
		
		'''
		
		if spewSegmentDebugData:	
			print( 'position/plotDistance: '+str(position/plotDistance))

		for index in range (0, len(distArray) ):
			#Scale our trajectory to the "actual" travel distance that we need:
			fractionalDistance = distArray[index] / position # Fractional position along the intended path
			destArray1.append ( int(round( fractionalDistance * motorSteps1)))
			destArray2.append ( int(round( fractionalDistance * motorSteps2)))

		prevMotor1 = 0
		prevMotor2 = 0
		prevTime = 0
		
		for index in range (0, len(destArray1) ):
			moveSteps1 = destArray1[index] - prevMotor1
			moveSteps2 = destArray2[index] - prevMotor2
			moveTime = durationArray[index] - prevTime
			prevTime = durationArray[index]

			if ( moveTime < 1 ):
				moveTime = 1		# don't allow zero-time moves.
	
			if (abs((float(moveSteps1) / float(moveTime))) < 0.002):	
				moveSteps1 = 0		#don't allow too-slow movements of this axis
			if (abs((float(moveSteps2) / float(moveTime))) < 0.002):	
				moveSteps2 = 0		#don't allow too-slow movements of this axis
	
			prevMotor1 += moveSteps1
			prevMotor2 += moveSteps2

			xSteps = (moveSteps1 + moveSteps2)/2.0	# Result will be a float.
			ySteps = (moveSteps1 - moveSteps2)/2.0	

			if ((moveSteps1 != 0) or (moveSteps2 != 0)): # if at least one motor step is required for this move....
	
				if (not self.resumeMode) and (not self.bStopped):
					ebb_motion.doXYMove( self.serialPort, moveSteps2, moveSteps1, moveTime )			
					if (moveTime > 15):
						time.sleep(float(moveTime - 10)/1000.0)  #pause before issuing next command
					else:
						if spewSegmentDebugData:	
							print( 'ShortMoves: ' + str( moveTime ) + '.' )

					self.fCurrX += xSteps / self.stepsPerInch   # Update current position
					self.fCurrY += ySteps / self.stepsPerInch		
	
					self.svgLastKnownPosX = self.fCurrX - axidraw_conf.StartPos_X
					self.svgLastKnownPosY = self.fCurrY - axidraw_conf.StartPos_Y	
					#if spewSegmentDebugData:			
					#	print( '\nfCurrX,fCurrY (x = %1.2f, y = %1.2f) ' % (self.fCurrX, self.fCurrY))
						
		strButton = ebb_motion.QueryPRGButton(self.serialPort)	#Query if button pressed
		if strButton[0] == '1': #button pressed
			self.svgNodeCount = self.nodeCount - 1;
			self.svgPausedPosX = self.fCurrX - axidraw_conf.StartPos_X	#self.svgLastKnownPosX
			self.svgPausedPosY = self.fCurrY - axidraw_conf.StartPos_Y	#self.svgLastKnownPosY
			self.penUp()
			print( 'Plot paused by button press after node number ' + str( self.nodeCount ) + '.' )
			print( 'Use the "resume" feature to continue.' )
			self.bStopped = True
			return
		
	def EnableMotors( self ):
		''' 
		Enable motors, set native motor resolution, and set speed scales.
		
		The "pen down" speed scale is adjusted with the following factors 
		that make the controls more intuitive: 
		* Reduce speed by factor of 2 when using 8X microstepping
		* Reduce speed by factor of 2 when disabling acceleration
		
		These factors prevent unexpected dramatic changes in speed when turning
		those two options on and off. 
		
		'''

		if (self.LayerOverrideSpeed):
			LocalPenDownSpeed = self.LayerPenDownSpeed
		else:	
			LocalPenDownSpeed = self.optionsPenDownSpeed

		if ( self.optionsResolution == 1 ):
			ebb_motion.sendEnableMotors(self.serialPort, 1) # 16X microstepping
			self.stepsPerInch = float( axidraw_conf.DPI_16X)						
			self.PenDownSpeed = LocalPenDownSpeed * axidraw_conf.Speed_Scale / 110.0
			self.PenUpSpeed = self.optionsRapidSpeed * axidraw_conf.Speed_Scale / 110.0
		elif ( self.optionsResolution == 2 ):
			ebb_motion.sendEnableMotors(self.serialPort, 2) # 8X microstepping
			self.stepsPerInch = float( axidraw_conf.DPI_16X / 2.0 )  
			self.PenDownSpeed = LocalPenDownSpeed * axidraw_conf.Speed_Scale / 220.0
			self.PenUpSpeed = self.optionsRapidSpeed * axidraw_conf.Speed_Scale / 110.0
		if (self.optionsConstSpeed):
			self.PenDownSpeed = self.PenDownSpeed / 2
		
		TestArray = array('i')	#signed integer
		if (TestArray.itemsize < 4):
			print( 'Internal array data length error. Please contact technical support.' )
			# This is being run on a system that has a shorter length for a signed integer
			# than we are expecting. If anyone ever comes across such a system, we need to know!
	
	def penUp( self ):
                print "penUp"
		self.virtualPenIsUp = True  # Virtual pen keeps track of state for resuming plotting.
		if ( not self.resumeMode) and (not self.bPenIsUp):	# skip if pen is already up, or if we're resuming.
			if (self.LayerOverridePenDownHeight):
				penDownPos = self.LayerPenDownPosition
			else:	
				penDownPos = self.optionsPenDownPosition
                                pass
			vDistance = float(self.optionsPenUpPosition - penDownPos)
			vTime = int ((1000.0 * vDistance) / self.optionsServoUpSpeed)
			if (vTime < 0):	#Handle case that penDownPosition is above penUpPosition
				vTime = -vTime
			vTime += self.optionsPenUpDelay	
			if (vTime < 0): #Do not allow negative delay times
				vTime = 0	
			ebb_motion.sendPenUp(self.serialPort, vTime )		
			if (vTime > 15):
				time.sleep(float(vTime - 10)/1000.0)  #pause before issuing next command
			self.bPenIsUp = True

	def penDown( self ):
                print "penDown"
		self.virtualPenIsUp = False  # Virtual pen keeps track of state for resuming plotting.
		if (self.bPenIsUp != False):  # skip if pen is already down
                        print "pen is not already down"
			if ((not self.resumeMode) and ( not self.bStopped )): #skip if resuming or stopped
                                print "not resuming or stopped"
				if (self.LayerOverridePenDownHeight):
					penDownPos = self.LayerPenDownPosition
				else:	
					penDownPos = self.optionsPenDownPosition
				vDistance = float(self.optionsPenUpPosition - penDownPos)
				vTime = int ((1000.0 * vDistance) / self.optionsServoDownSpeed)
				if (vTime < 0):	#Handle case that penDownPosition is above penUpPosition
					vTime = -vTime
				vTime += self.optionsPenDownDelay	
				if (vTime < 0): #Do not allow negative delay times
					vTime = 0
                                print "about to send pen down instruction", vTime
				ebb_motion.sendPenDown(self.serialPort, vTime )						
				if (vTime > 15):
					time.sleep(float(vTime - 10)/1000.0)  #pause before issuing next command
				self.bPenIsUp = False

	def ServoSetupWrapper( self ):
		# Assert what the defined "up" and "down" positions of the servo motor should be,
		#    and determine what the pen state is.
		self.ServoSetup()
		strVersion = ebb_serial.query( self.serialPort, 'QP\r' )
		if strVersion[0] == '0':
			self.bPenIsUp = False
		else:
			self.bPenIsUp = True

	def ServoSetup( self ):
		''' Pen position units range from 0% to 100%, which correspond to
		    a typical timing range of 7500 - 25000 in units of 1/(12 MHz).
		    1% corresponds to ~14.6 us, or 175 units of 1/(12 MHz).
		'''

		if (self.LayerOverridePenDownHeight):
			penDownPos = self.LayerPenDownPosition
		else:	
			penDownPos = self.optionsPenDownPosition
		
		servo_range = axidraw_conf.SERVO_MAX - axidraw_conf.SERVO_MIN
		servo_slope = float(servo_range) / 100.0
		
		intTemp = int(round(axidraw_conf.SERVO_MIN + servo_slope * self.optionsPenUpPosition))
		ebb_serial.command( self.serialPort,  'SC,4,' + str( intTemp ) + '\r' )	
				
		intTemp = int(round(axidraw_conf.SERVO_MIN + servo_slope * penDownPos))
		ebb_serial.command( self.serialPort,  'SC,5,' + str( intTemp ) + '\r' )

		''' Servo speed units are in units of %/second, referring to the
			percentages above.  The EBB takes speeds in units of 1/(12 MHz) steps
			per 24 ms.  Scaling as above, 1% of range in 1 second 
			with SERVO_MAX = 28000  and  SERVO_MIN = 7500
			corresponds to 205 steps change in 1 s
			That gives 0.205 steps/ms, or 4.92 steps / 24 ms
			Rounding this to 5 steps/24 ms is sufficient.		'''
		
		intTemp = 5 * self.optionsServoUpSpeed
		ebb_serial.command( self.serialPort, 'SC,11,' + str( intTemp ) + '\r' )

		intTemp = 5 * self.optionsServoDownSpeed
		ebb_serial.command( self.serialPort,  'SC,12,' + str( intTemp ) + '\r' )

	def stop( self ):
		self.bStopped = True

        def setupSerial(self):
 		self.serialPort = ebb_serial.openPort()
 		if self.serialPort is None:
			print gettext.gettext( "Failed to connect to AxiDraw. :(")
                        return False
                return True

	def getDocProps( self ):
		'''
		Get the document's height and width attributes from the <svg> tag.
		Use a default value in case the property is not present or is
		expressed in units of percentages.
		'''
		self.svgHeight = plot_utils.getLengthInches( self, 'height' )
		self.svgWidth = plot_utils.getLengthInches( self, 'width' )
		if (self.optionsAutoRotate) and (self.svgHeight > self.svgWidth ):
			self.printPortrait = True
		if ( self.svgHeight == None ) or ( self.svgWidth == None ):
			return False
		else:
			return True
