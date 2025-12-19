class Vector3D():
    """Classe vecteur 3D (x,y,z)"""
    def __init__(self,x=0,y=0,z=0):
        """Constructeur avec des valeur par défaut nulles"""
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return "Vector3D(%g, %g, %g)" % (self.x, self.y, self.z)
        
    def __repr__(self):
        return "Vector3D(%g, %g, %g)" % (self.x, self.y, self.z)

    def __add__(self,other):
        return Vector3D(self.x + other.x, self.y + other.y, self.z + other.z)

    def __neg__(self):
        return Vector3D(-self.x,-self.y,-self.z)
    
    def __sub__(self,other):
        return self + (-other)

    def __mul__(self,other):
        """ Vectoriel entre 2 vecteurs (*) """
        if type(other) is Vector3D:
            X = self.y*other.z - self.z*other.y
            Y = self.z*other.x - self.x*other.z
            Z = self.x*other.y - self.y*other.x
        else:
            X= other * self.x
            Y= other * self.y
            Z= other * self.z

        return Vector3D(X,Y,Z)

    def __rmul__(self,other):
        return self*other

    def __pow__(self,other):
        """ scalaire des 2 vecteurs (**)"""
        if type(other) is Vector3D:
            return (self.x * other.x +self.y * other.y +self.z * other.z)
        
        else:
            X= other * self.x
            Y= other * self.y
            Z= other * self.z
            return Vector3D(X,Y,Z)

    def __rpow__(self,other): 
        return self**other      

    def __eq__(self,other):
        if (self.x == other.x and self.y == other.y and self.z == other.z):
            return True
        else:
            return False
        
    def mod(self):
        """La norme du Vector3D"""
        return (self**self)**.5

    def norm(self):
        """Vecteur Normalisé"""
        m = self.mod()
        if m != 0:
            return self * (1/m)
        else: 
            return Vector3D()

    def rotZ(self,theta):
        from numpy import cos, sin
        
        x = cos(theta)*self.x - sin(theta)*self.y
        y = cos(theta)*self.y + sin(theta)*self.x
        
        self.x = x
        self.y = y
        return self

    def save(self,nom='vec.dat'):
        from pickle import dump
        file = open(nom,'wb')
        dump(self,file)
        file.close

    def load(self,nom='vec.dat'):
        from pickle import load
        file = open(nom,'rb')
        temp=load(file)
        self.x = temp.x
        self.y = temp.y
        self.z = temp.z
        file.close

        
    

if __name__ == "__main__": # false lors d'un import

    from numpy import pi
    
    v0=Vector3D()
    v1 = Vector3D(1,0,0)
    v2 = Vector3D(0,1,0)
    v3 = Vector3D(0,0,1)
    v5 = Vector3D(5,7,-5)
    print(v1*v2)
    print(v2*v1)
    print(v1*v2 == v3)
    print((v1*v2)-v3 == v0)

    print(v1 * 5)
    print(5*v1)

    m = v5.mod()
    n = v5.norm()

    print(m,n)
    print(n.mod())

    print(m*n == v5)

    print(v1.rotZ(pi/2))

    
    v1.save('test.vec')

    v2.load('test.vec')

    from pickle import load

    f=open('test.vec','rb')

    vL = load(f)
    
    print(v1,vL,v3)
        

    
