import numpy as np
from sklearn import linear_model, svm
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
n_samples, n_features = 10, 5
f=open('datalog.csv','r')
data=f.read()
data=data.split('\n')
data_array=[]
line_counter=0
for line in data:
	line_counter += 1
	for entry in line.split(','):
		data_array.append(entry)
line_counter += -1
data_array=data_array[0:7*line_counter]
data=np.array(data_array)
data=data.reshape(line_counter,7)
data=data.astype(float)
y = data[:,0]#np.random.randn(n_samples)
X = data[:,3:7]#.reshape(line_counter,1)#np.random.randn(n_samples, n_features)
print(y,X)
X_train, X_test, y_train, y_test = train_test_split(X,y, test_size=0.2, random_state=0)
#clf = linear_model.Ridge (alpha = .5)
clf=linear_model.Lasso()
clf.fit(X_train, y_train)
test_results=[]
for x,y in zip(X_test,y_test):
	#print(list(x),clf.predict(x.reshape(1,-1))[0],y)
	test_results.append([y,clf.predict(x.reshape(1,-1))[0]])
test_results=sorted(test_results)
print(clf.score(X_train,y_train))
test_results=np.array(test_results)
#print(test_results[:,1],test_results.shape)
plt.plot(test_results[:,0],test_results[:,1])
plt.show()