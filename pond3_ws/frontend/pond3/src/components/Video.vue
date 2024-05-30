<template>
    <canvas ref="canvas" width="440" height="380"></canvas>
    <p>Latência: {{latency}} ms</p>
</template>

<script>
export default {
  name: 'Video',

  data() {
    return {
      ws: null,
      latency : 0
    }
  },

  mounted() {
    this.ws = new WebSocket('ws://localhost:8500/wsVideo');
    // GArantindo que o websocket tá enviando um binário 
    this.ws.binaryType = 'arraybuffer';
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
      
      // Pegando a array que vem do websocket convertendo pra string e pegando o timestamp do back
      const uint8Array = new Uint8Array(arrayBuffer);
      
      const dataString = new TextDecoder().decode(uint8Array);
      
      const parts = dataString.split("::");

      const timestampString = parts[1];

      // Convertendo o timestamp para número 
      const timestamp = Number(timestampString);

      // Calculate the latency
      const latencyCalc= ((Date.now() - timestamp)/1000);
      const roundedLatencySeconds = latencyCalc.toFixed(2);
      this.latency = roundedLatencySeconds;



      const image = new Image();
      image.onload = () => {
        context.drawImage(image, 0, 0, canvas.width, canvas.height); 
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