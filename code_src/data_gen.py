import numpy as np
from scipy.spatial.transform import Rotation as R

def write_matrix2file(f, a):
    mat = np.matrix(a)
    for line in mat:
        np.savetxt(f, line, fmt='%.5f')
    f.write("\n")

def main():
    #AX = ZB data generator
    n = 10
    r = R.from_euler('zyx', [30,50,60], degrees=True)
    #r = R.from_rotvec(np.pi/2 * np.array([1, 0, 1]))
    Rx = r.as_matrix()
    X = np.block([[Rx, np.random.rand(3,1)],[0, 0, 0, 1]])
    print("X", X)
    #r = R.from_rotvec(np.pi/2 * np.array([0, 1, 1]))
    r = R.from_euler('zyx', [20,10,80], degrees=True)
    Rz = r.as_matrix()
    Z = np.block([[Rz, np.random.rand(3,1)],[0, 0, 0, 1]])
    print("Z", Z)

    fa = open("As.txt", "w")
    fb = open("Bs.txt", "w")
    fa.write(str(n)+"\n")
    fb.write(str(n)+"\n")
    fc = open("X_Z.txt", "w")
    fc.write("X: \n")
    write_matrix2file(fc, X)
    fc.write("X-1: \n")
    write_matrix2file(fc, np.linalg.inv(X))
    fc.write("Z: \n")
    write_matrix2file(fc, Z)

    for i in range(n):
        r = R.from_rotvec(3 * np.random.rand() * np.random.rand(3))
        A = np.block([[r.as_matrix(), np.random.rand(3,1)],[0, 0, 0, 1]])
        B = np.linalg.inv(Z).dot(A).dot(X)
    
        #write to file 
        write_matrix2file(fa, A)
        write_matrix2file(fb, B)

        err = np.linalg.norm(A.dot(X) - Z.dot(B))
        print("err: ", err)
        

    fa.close()
    fb.close()
    fc.close()

if __name__ == "__main__":
    main()