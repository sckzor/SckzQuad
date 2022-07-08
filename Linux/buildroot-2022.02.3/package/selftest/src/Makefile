.PHONY: clean

selftest: selftest.c smbus.o i2c.o pwm.o gyro.o madgwick.o
	$(CC) -o '$@' $^

madgwick.o: madgwick.c $(DEPS)
	$(CC) -c -o '$@' '$<'

gyro.o: gyro.c $(DEPS)
	$(CC) -c -o '$@' '$<'

pwm.o: pwm.c $(DEPS)
	$(CC) -c -o '$@' '$<'

i2c.o: i2c.c $(DEPS)
	$(CC) -c -o '$@' '$<'

smbus.o: smbus.c $(DEPS)
	$(CC) -c -o '$@' '$<'

clean:
	rm selftest smbus.o i2c.c pwm.c gyro.o madgwick.o
