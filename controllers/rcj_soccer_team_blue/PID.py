
class PID():
    I=0
    y=[0,0]
    def __init__(self,kd,kp,ki,T) :
        self.kd=kd
        self.kp=kp
        self.ki=ki
        self.T=T
    def run(self,x):
        self.y[0]=self.y[1]
        self.y[1]=self.kp*x[1]+I+(self.kd/self.T)*(x[1]-x[0])
        I=self.ki*self.T*x[1]+I
        return self.y
        
