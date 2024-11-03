import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

# 定義範圍和常態分佈參數
x = np.linspace(-5, 5, 500)
mu, sigma = 0, 1

# 計算 f(x) 和 g(x)
f_x = norm.pdf(x, mu, sigma)
g_x = np.log(f_x)

# 圖 1: f(x) 和 g(x)
plt.figure(figsize=(10, 6))
plt.plot(x, f_x, label=r'$f(x)$ - Normal Distribution')
plt.plot(x, g_x, label=r'$g(x) = \ln(f(x))$')
plt.xlabel('x')
plt.ylabel('Value')
plt.title('Normal Distribution and its Logarithmic Transformation')
plt.legend()
plt.grid(True)
plt.show()

# 計算 f'(x) 和 g'(x)
f_prime_x = -x * f_x / sigma**2  # derivative of normal PDF
g_prime_x = 0.1*f_prime_x / f_x       # derivative of log of normal PDF

# 圖 2: f'(x) 和 g'(x)
plt.figure(figsize=(10, 6))
plt.plot(x, f_prime_x, label=r"$f'(x)$ - Derivative of Normal Distribution")
plt.plot(x, g_prime_x, label=r"$g'(x) = \frac{f'(x)}{f(x)}$")
plt.xlabel('x')
plt.ylabel('Derivative Value')
plt.title("Derivatives of Normal Distribution and its Logarithmic Transformation")
plt.legend()
plt.grid(True)
plt.show()
