import numpy as np
from sklearn import linear_model, svm
from sklearn.model_selection import train_test_split
from sklearn.externals import joblib
import matplotlib.pyplot as plt
n_samples, n_features = 10, 5
f=open('datalog_no_spin.csv','r')
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
y = data[:,0]
X = data[:,3:7]
##################
##Teting Models###
##################
"""print(y.shape,X.shape)
X_train, X_test, y_train, y_test = train_test_split(X,y, test_size=0.2, random_state=0)
#clf = linear_model.Ridge (alpha = .5)
clf=linear_model.Lasso()
clf.fit(X_train, y_train)
test_results=[]
for x,y in zip(X_test,y_test):
	test_results.append([y,clf.predict(x.reshape(1,-1))[0]])
test_results=sorted(test_results)
print(clf.score(X_train,y_train))
test_results=np.array(test_results)
plt.plot(test_results[:,0],test_results[:,1])
plt.show()"""
##################
##Saving Models###
##################
#axis 1
y = data[:,0]
X = data[:,3:7]
#model_1=linear_model.Ridge (alpha = .5)
model_1=linear_model.Lasso()
model_1.fit(X,y)
filename = 'axis1_model.sav'
joblib.dump(model_1, filename)
#axis 2
y = data[:,1]
X = data[:,3:7]
#model_2=linear_model.Ridge (alpha = .5)
model_2=linear_model.Lasso()
model_2.fit(X,y)
filename = 'axis2_model.sav'
joblib.dump(model_2, filename)
for x,y in zip(X,y):
	print(x.reshape(1,-1))