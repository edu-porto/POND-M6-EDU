import numpy as np



class MultiLayerPerceptron:
    def __init__(self, hidden_size, learning_rate=0.2):
        self.input_size = 2
        self.hidden_size = hidden_size
        self.output_size = 1 
        self.learning_rate = learning_rate

        # É preciso que a propría rede seja capaz de criar os pesos e bias 
        # Neste caso o usuário pode definir a quantia de camadas escondidas e o modelo vai se ajustando conforme necessário 

        self.weights = []
        self.bias = []

        # Primeiro input para a camada escondida 
        self.weights.append(np.random.randn(self.input_size, self.hidden_size[0]))
        self.bias.append(np.zeros((1, self.hidden_size[0])))

        # Camadas escondidas
        for i in range(1, len(self.hidden_size)):
            self.weights.append(np.random.randn(self.hidden_size[i-1], self.hidden_size[i]))
            self.bias.append(np.zeros((1, self.hidden_size[i])))

        # Hidden to output layer
        self.weights.append(np.random.randn(self.hidden_size[-1], self.output_size))
        self.bias.append(np.zeros((1, self.output_size)))


    # tive que trocar pra numpy 
    def _sigmoid(self, x):
        """
        Implementa a função sigmoide
        Essa é uma função de ativação possível para os nós da rede neural.
        """
        return 1/(1 + np.exp(-x))
    
    def _sigmoid_derivative(self, x):
        return x * (1 - x)

    def forward_pass(self, data):
        self.layer_inputs = [data]
        self.layer_outputs = [data]

        for x in range(len(self.hidden_size)-1):
            layer_input = np.dot(self.layer_outputs[-1], self.weights[x]) + self.bias[x]
            layer_output = self._sigmoid(layer_input)
            self.layer_inputs.append(layer_input)
            self.layer_outputs.append(layer_output)

        # Aplicando as mudanças para a camada de saída
        output_layer_input = np.dot(self.layer_outputs[-1], self.weights[-1]) + self.bias[-1]
        output_layer_output = self._sigmoid(output_layer_input)
        self.layer_inputs.append(output_layer_input)
        self.layer_outputs.append(output_layer_output)


        return output_layer_output
    
    def backpropagation(self, x_input, y_output):
        # Aplicando o forward pass
        output = self.forward_pass(x_input)

        # Calculando o erro

        error_back = [y_output - output]
        delta_error = [error_back[0] * self._sigmoid_derivative(output)]

        # Fazendo o backpropagation 
        for x in range(len(self.hidden_size), 0, -1):
            error_loop = np.dot(delta_error[0], self.weights[x].T)
            delta_loop = error_loop * self._sigmoid_derivative(self.layer_outputs[x])
            error_back.insert(0, error_loop)
            delta_error.insert(0, delta_loop)

        # Atualizando os pesos e bias 
        for x in range(len(self.hidden_size) + 1):
            self.weights[x] += self.learning_rate * np.dot(self.layer_outputs[x].reshape(-1, 1), delta_error[x].reshape(1, -1))
            self.bias[x] += self.learning_rate * delta_error[x]


    
    def train(self, X_train, y_train, epochs):
        self.epochs = epochs
        for i in range(epochs):
            for input, target in zip(X_train, y_train):
                self.backpropagation(input, target)

    def prediction(self, user_inputs):
        return self.forward_pass(user_inputs)

def main():
    training_inputs = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])
    training_targets = np.array([[0], [1], [1], [0]])

    # Como o problema é do XOR o usuário só consegue definir o número de camadas escondidas 
    el_mlp = MultiLayerPerceptron([2])
    el_mlp.train(training_inputs, training_targets, 4521)

    # testando a predição
    Y_pred = [1,0]
    prediction = el_mlp.prediction(Y_pred) 
    print("Quanto mais próximo de 1 mais provável que seja uma porta XOR")
    print("Quanto mais próximo de 0 mais provável que não seja uma porta XOR")
    print(f'input : {Y_pred} -> output : {prediction}')



if __name__ == "__main__":
   main()