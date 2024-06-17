from flask import Flask, render_template, request, redirect, url_for,jsonify, send_from_directory
import tensorflow as tf
import numpy as np
from werkzeug.utils import secure_filename
import os
from PIL import Image

app = Flask(__name__)

# Create a folder to save uploaded images
UPLOAD_FOLDER = 'uploads'
if not os.path.exists(UPLOAD_FOLDER):
    os.makedirs(UPLOAD_FOLDER)

app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER


@app.route('/')
def index():
    return render_template('index.html')

# Carrengando o modelo 
model = tf.keras.models.load_model('../models/mnist_cnn.h5')

# Função que faz o processamento da imagem pro modelo fazer a predição 
def preprocess_image(img_path):
    img = Image.open(img_path).convert('L')  
    img = img.resize((28, 28))  
    img = np.array(img)  
    img = img / 255.0  
    img = img.reshape(1, 28, 28, 1)  
    return img

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

@app.route('/predict/<filename>')
def predict(filename):
    file_path = os.path.join(app.config['UPLOAD_FOLDER'], filename)
    img = preprocess_image(file_path)
    prediction = model.predict(img)
    prediction_class = np.argmax(prediction, axis=1)[0]
    return render_template('result.html', filename=filename, prediction_class=prediction_class)

@app.route('/uploads/<filename>')
def uploaded_file(filename):
    return send_from_directory(app.config['UPLOAD_FOLDER'], filename)


if __name__ == '__main__':
    app.run(debug=True)