.PHONY: clean

selftest: selftest.c smbus.o gyro.o pwm.o madgwick.o
	$(CC) -o '$@' $^

gyro.o: gyro.c $(DEPS)
	$(CC) -c -o '$@' '$<'

pwm.o: pwm.c $(DEPS)
	$(CC) -c -o '$@' '$<'

madgwick.o: madgwick.c $(DEPS)
	$(CC) -c -o '$@' '$<'

smbus.o: smbus.c $(DEPS)
	$(CC) -c -o '$@' '$<'

clean:
	rm selftest smbus.o gyro.o pwm.o madgwick.o
