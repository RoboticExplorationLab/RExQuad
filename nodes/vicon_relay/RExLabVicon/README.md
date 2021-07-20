# RExLabVicon

Builds a simple executable which runs a ZMQ publisher to publish Vicon streaming data.

To build & run do the following:

```bash
./build.sh

#./build/vicon_pub  <publisher_ip>  <vicon_body_name>  <subscriber_ip>  <subscriber_port>
./build/vicon_pub  192.168.3.249  unitreea1  127.0.0.1  5000
```

