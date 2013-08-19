#!/usr/bin/env python2
import numpy as np

if __name__ == '__main__':
	# test data
	R1 = np.matrix('1,0,0;0,1,0;0,0,1')
	R2 = np.matrix('1,0,0;0,1,0;0,0,-1')
	R3 = np.matrix('0,0,1;0,1,0;-1,0,0')
	R4 = np.matrix('1,0,0;0,0,1;0,-1,0')

	F1 = np.matrix('1;2;-26.43')
	F2 = np.matrix('1;2;32.43')
	F3 = np.matrix('-28.43;2;3')
	F4 = np.matrix('1;-27.43;3')

	M1 = np.matrix('-1;27.43;3')
	M2 = np.matrix('-1;-31.43;3')
	M3 = np.matrix('-1;-2;3')
	M4 = np.matrix('-1;-2;-26.43')

	g = 9.81

	#1
	A = np.matrix([[1,0,0,-R1[0,2]*g]])
	A = np.vstack([A,[0,1,0,-R1[1,2]*g]])
	A = np.vstack([A,[0,0,1,-R1[2,2]*g]])
	b = F1
	#2
	A = np.vstack([A,[1,0,0,-R2[0,2]*g]])
	A = np.vstack([A,[0,1,0,-R2[1,2]*g]])
	A = np.vstack([A,[0,0,1,-R2[2,2]*g]])
	b = np.vstack([b,F2])
	#3
	A = np.vstack([A,[1,0,0,-R3[0,2]*g]])
	A = np.vstack([A,[0,1,0,-R3[1,2]*g]])
	A = np.vstack([A,[0,0,1,-R3[2,2]*g]])
	b = np.vstack([b,F3])
	#4
	A = np.vstack([A,[1,0,0,-R4[0,2]*g]])
	A = np.vstack([A,[0,1,0,-R4[1,2]*g]])
	A = np.vstack([A,[0,0,1,-R4[2,2]*g]])
	b = np.vstack([b,F4])
	print 'A =\n' + str(A)
	print 'b =\n' + str(b)

	#vypocet F
	U,s,V = np.linalg.svd(A,full_matrices=1)
	V = V.getH()
	S = np.zeros(shape=(len(b),4))
	S[0,0] = s[0]
	S[1,1] = s[1]
	S[2,2] = s[2]
	S[3,3] = s[3]
	X = V*np.linalg.pinv(S)*U.getH()*b
	print 'Xf =\n' + str(X)
	Fo = X[0:2]
	m = X[3]
	#1
	A = np.matrix([[1,0,0,0,-m*R1[2,2]*g,m*R1[1,2]*g]])
	A = np.vstack([A,[0,1,0,m*R1[2,2]*g,0,-m*R1[0,2]*g]])
	A = np.vstack([A,[0,0,1,-m*R1[1,2]*g,m*R1[0,2]*g,0]])
	b = M1
	#2
	A = np.vstack([A,[1,0,0,0,-m*R2[2,2]*g,m*R2[1,2]*g]])
	A = np.vstack([A,[0,1,0,m*R2[2,2]*g,0,-m*R2[0,2]*g]])
	A = np.vstack([A,[0,0,1,-m*R2[1,2]*g,m*R2[0,2]*g,0]])
	b = np.vstack([b,M2])
	#3
	A = np.vstack([A,[1,0,0,0,-m*R3[2,2]*g,m*R3[1,2]*g]])
	A = np.vstack([A,[0,1,0,m*R3[2,2]*g,0,-m*R3[0,2]*g]])
	A = np.vstack([A,[0,0,1,-m*R3[1,2]*g,m*R3[0,2]*g,0]])
	b = np.vstack([b,M3])
	#4
	A = np.vstack([A,[1,0,0,0,-m*R4[2,2]*g,m*R4[1,2]*g]])
	A = np.vstack([A,[0,1,0,m*R4[2,2]*g,0,-m*R4[0,2]*g]])
	A = np.vstack([A,[0,0,1,-m*R4[1,2]*g,m*R4[0,2]*g,0]])
	b = np.vstack([b,M4])
	print 'A =\n' + str(A)
	print 'b =\n' + str(b)

	#vypocet M
	U,s,V = np.linalg.svd(A,full_matrices=1)
	V = V.getH()
	S = np.zeros(shape=(len(b),6))
	S[0,0] = s[0]
	S[1,1] = s[1]
	S[2,2] = s[2]
	S[3,3] = s[3]
	S[4,4] = s[4]
	S[5,5] = s[5]
	X = V*np.linalg.pinv(S)*U.getH()*b
	print 'Xf =\n' + str(X)