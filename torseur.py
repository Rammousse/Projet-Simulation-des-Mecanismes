class Torseur():
    """Classe Torseur(P,R,M), [P]oint, [R]ésultante, [M]oment"""

    from vector3D import Vector3D as v

    def __init__(self,P=v(),R=v(),M=v()):
        self.P = P
        self.R = R
        self.M = M

    def __str__(self):
        return 'Torseur(%s, %s, %s)' % (self.P, self.R, self.M)

    def __repr__(self):
        return str(self)


    def changePoint(self,Po=v()):
        MD = self.M + (self.P - Po) * self.R
        self.P = Po
        self.M = MD

    def __add__(self,other):

        Po = other.P
        other.changePoint(self.P)
        T = Torseur(self.P,self.R+other.R,self.M+other.M)
        other.changePoint(Po)
        return(T)

    def __neg__(self):
        return Torseur(self.P,-self.R,-self.M)

    def __sub__(self,other):
        return(self + (-other))

    def __eq__(self,other):
        Po = other.P
        other.changePoint(self.P)
        test = self.R == other.R and self.M==other.M
        other.changePoint(Po)
        return(test)



if __name__ == "__main__": # false lors d'un import
    from vector3D import Vector3D as v

    P= v()
    R = v(1,0,0)
    M=v(0,1,0)

    T = Torseur(P,R,M)
    T2 = Torseur(M,P,R)

    print(T,'\n+\n',T2,'\n=\n',T+T2)

    T.changePoint(v(1,1,0))

    print('T au point (1,1,0)=',T)
