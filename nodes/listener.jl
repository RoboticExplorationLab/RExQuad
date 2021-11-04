using ZMQ

# %%
ctx = ZMQ.Context(1)
s1 = Socket(ZMQ.SUB)
ZMQ.subscribe(s1)
ZMQ.connect(s1, "tcp://127.0.0.1:5558")

# %%
msg = ZMQ.recv(s1)
display(msg)