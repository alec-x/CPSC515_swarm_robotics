# Examples borrowed from: 
# https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.levy.html
# https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.cauchy.html

from scipy.stats import cauchy
from scipy.stats import levy
import matplotlib.pyplot as plt
import numpy as np

fig, ax = plt.subplots(1, 2, constrained_layout=True)
x_cauchy = np.linspace(cauchy.ppf(0.01),
                cauchy.ppf(0.99), 100)
x_levy = np.linspace(levy.ppf(0.05),
                levy.ppf(0.95), 100)

ax[0].plot(x_cauchy, cauchy.pdf(x_cauchy),
       'b-', lw=3, alpha=0.8, label='cauchy pdf')
ax[0].set_title('Cauchy Distribution')


ax[1].plot(x_levy, levy.pdf(x_levy),
       'r-', lw=3, alpha=0.8, label='levy pdf')
ax[1].set_title('Levy Distribution')

plt.show()