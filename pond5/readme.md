# Atividade ponderada - 3

## Objetivo 

Desenvolver um página de frontend, conectada a um servidor flask que retorne a classificação de um número com o dataset mnist. 

## Atividades Desenvolvidas 

### Backend
O backend permite a conexão do modelo com a página web para input.

O código pode ser acessado em: 

    cd pond5/web/main.py

Na rota a seguir, é possível enviar imagens para o backend e, a partir do momento em que o usuário confirma, o upload é realizado e a imagem é classificada.

```python
@app.route('/upload', methods=['POST'])
def upload_image():
    if 'image' not in request.files:
        return redirect(request.url)
    file = request.files['image']
    if file.filename == '':
        return redirect(request.url)
    if file:
        filename = secure_filename(file.filename)
        file_path = os.path.join(app.config['UPLOAD_FOLDER'], filename)
        file.save(file_path)
        return redirect(url_for('predict', filename=filename))
```

A rota de predição realiza os tratamentos necessários no arquivo e retorna o número que o modelo acredita que aquele input representa.
```python
@app.route('/predict/<filename>')
def predict(filename):
    file_path = os.path.join(app.config['UPLOAD_FOLDER'], filename)
    img = preprocess_image(file_path)
    prediction = model.predict(img)
    prediction_class = np.argmax(prediction, axis=1)[0]
    return render_template('result.html', filename=filename, prediction_class=prediction_class)
```

Esta rota é responsável por retornar no frontend a imagem no qual foi realizada a predição. 
```python
@app.route('/uploads/<filename>')
def uploaded_file(filename):
    return send_from_directory(app.config['UPLOAD_FOLDER'], filename)

```

### Rede neural com tensor flow 
Foi criado uma rede neural convulucional.

O código pode ser acessado em: 

    cd pond5/notebooks/mnist.ipynb

### Frontend

Foram criadas duas páginas em HTML simples. 

A primeira é o index do projeto e pode ser acessado em: 

    cd pond5/web/templates/index.html

A segunda página é a de resultados da predição e pode ser acessada em: 

    cd pond5/web/templates/result.html




## Como utilizar a solução 


1. Na pasta web é preciso criar uma venv e instalar os requirements.

    ```console 
        python -m venv venv
        cd venv/scripts
        activate
        pip install -r requirements.txt
    ``` 

2. Rodar o servidor flask.

    ```console 
        python main.py
    ``` 

## Demonstração dos trabalhos realizados 

Na figura abaixo é possível assistir o funcionamento da atividade desenvolvida. 

[![Demonstração](https://img.youtube.com/vi/JUH4Y-Vu4mY/0.jpg)](https://www.youtube.com/watch?v=JUH4Y-Vu4mY)


