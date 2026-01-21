import matplotlib.pyplot as plt
import pickle

# load q from file
with open("q.pkl", "rb") as f:
    q = pickle.load(f)


plt.figure()
for i in range(len(q)):
    qp = q[i]
    plt.plot(range(len(qp)), qp)
# plt.plot(range(len(q[0])), q[0])
# plt.plot(range(len(q[1])), q[1])
plt.show()

