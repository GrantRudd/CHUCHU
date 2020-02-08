#PID regulator implimentation
class PID (object):
	def __init__(self, Kp, Kd, Ki, scaleFactor = 1):
		#PID gains
		self.Kp = Kp
		self.Kd = Kd
		self.Ki = Ki
		#integral term
		self.I = 0
		#last error term
		self.lastError = 0
		#Scale factor (default to 1)
		self.scaleFactor = scaleFactor

	def update_value(self, goal, current):
		#calculate error term
		error = goal - current
		#calculate dError
		dError = error - self.lastError
		#calculate integral term
		self.I = self.I + error

		#update command value
		commanded = (self.Kp*error + self.Kd*dError + self.Ki*self.I) * self.scaleFactor
		#set last error equal to the current error
		self.lastError = error

		return commanded, error


