
class Observer:
    def update(self,subject):
        raise NotImplementedError

class Subject(object):
    def __init__(self):
        self.observers=[]

    def notify(self):
        for obs in self.observers:
            obs.update(self)

    def attach(self,obs):
        if not callable(getattr(obs,"update")) :
            raise ValueError("Observer must have an update() method")
        self.observers.append(obs)
        obs.update(self)

    def detach(self,obs):
        if obs in self.observers :
            self.observers.remove(obs)