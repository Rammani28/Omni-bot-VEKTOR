import matplotlib.pyplot as plt

# Create time points (assuming each entry is 1 second)
time = list(range(len(r1)))

# Plot RPMs vs Targets
plt.figure(figsize=(12, 6))

# Plot r1 vs u1
plt.subplot(3, 1, 1)
plt.plot(time, r1, label='r1 (Current RPM)')
plt.plot(time, u1, label='u1 (Target RPM)', linestyle='--')
plt.ylabel('RPM')
plt.title('Motor 1 RPM vs Target')
plt.legend()

# Plot r2 vs u2
plt.subplot(3, 1, 2)
plt.plot(time, r2, label='r2 (Current RPM)')
plt.plot(time, u2, label='u2 (Target RPM)', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('RPM')
plt.title('Motor 2 RPM vs Target')
plt.legend()

# Plot r3 vs u3
plt.subplot(3, 1, 3)
plt.plot(time, r3, label='r3 (Current RPM)')
plt.plot(time, u3, label='u3 (Target RPM)', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('RPM')
plt.title('Motor 3 RPM vs Target')
plt.legend()

plt.tight_layout()
plt.show()
