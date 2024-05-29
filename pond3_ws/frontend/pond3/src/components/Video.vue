<template>
  <h1>Video</h1>
    <canvas ref="canvas" width="440" height="380"></canvas>
</template>

<script>
export default {
  name: 'Video',

  data() {
    return {
      ws: null
    }
  },

  mounted() {
    this.ws = new WebSocket('ws://localhost:8000/wsVideo');
    this.ws.onopen = () => {
      console.log('Connected to video server');
    };
    this.ws.onerror = (error) => {
      console.error('Video error: ', error);
    };
    this.ws.onmessage = (event) => {
      const canvas = this.$refs.canvas;
      const context = canvas.getContext('2d');

      const arrayBuffer = event.data;
      const blob = new Blob([arrayBuffer], { type: 'image/jpeg' });
      const url = URL.createObjectURL(blob);
      
      const image = new Image();
      image.onload = () => {
        // context.clearRect(0, 0, canvas.width, canvas.height);  // Clear the canvas
        context.drawImage(image, 0, 0, canvas.width, canvas.height);  // Corrected arguments
        URL.revokeObjectURL(url);
      };
      // image.src = 'data:image/jpeg;base64,' + event.data;
      image.src = url;
    };
  },
  beforeDestroy(){
    if (this.ws) {
      this.ws.close();
    }
  }
}

</script>


<style>
</style>