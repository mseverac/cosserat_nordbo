import matplotlib.pyplot as plt
from matplotlib.widgets import Button

fig, ax = plt.subplots()
line, = ax.plot([0, 1], [0, 1], label="Ma ligne")

leg = ax.legend(["Voici ce que je veux écrire"], loc="upper left")

# Callback pour masquer/afficher
def toggle_legend(event):
    vis = leg.get_visible()
    leg.set_visible(not vis)
    plt.draw()

# Créer un bouton
button_ax = plt.axes([0.8, 0.01, 0.1, 0.05])  # [left, bottom, width, height]
button = Button(button_ax, 'Légende')
button.on_clicked(toggle_legend)

plt.show()




[-1.36694682e+01  5.05954474e-02 -2.21993393e+00 -1.51874803e-01
  1.63691334e+00 -1.10965708e-02]