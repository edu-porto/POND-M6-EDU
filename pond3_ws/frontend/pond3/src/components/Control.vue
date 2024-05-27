<template>
  <div class="box-content h-64 w-64 p-4">
    <div class="flex items-center justify-center">
            <button :class="{'clicked': isForwardClicked }"><svg fill="#000000" height="100px" width="100px" version="1.1" id="Icons" xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" viewBox="0 0 32 32" xml:space="preserve" transform="matrix(1, 0, 0, 1, 0, 0)"><g id="SVGRepo_bgCarrier" stroke-width="0"></g><g id="SVGRepo_tracerCarrier" stroke-linecap="round" stroke-linejoin="round"></g><g id="SVGRepo_iconCarrier"> <g> <path d="M24.4,24c-0.9,0-1.8-0.3-2.5-1l-5.8-5.7c-0.1-0.1-0.2-0.1-0.3,0L10.1,23c-1.4,1.4-3.6,1.4-5,0c-0.7-0.7-1-1.6-1-2.5 c0-1,0.4-1.8,1-2.5l9.4-9.3c0.9-0.9,2.3-0.9,3.1,0l9.4,9.3c0.7,0.7,1,1.6,1,2.5c0,1-0.4,1.8-1,2.5C26.2,23.7,25.3,24,24.4,24z"></path> </g> </g></svg></button>
    </div>
    <div class="flex items-stretch justify-between">
      <button :class="{ 'clicked': isLeftClicked }"><svg fill="#000000" height="100px" width="100px" version="1.1" id="Icons" xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" viewBox="0 0 32 32" xml:space="preserve" transform="matrix(1, 0, 0, 1, 0, 0)rotate(270)"><g id="SVGRepo_bgCarrier" stroke-width="0"></g><g id="SVGRepo_tracerCarrier" stroke-linecap="round" stroke-linejoin="round"></g><g id="SVGRepo_iconCarrier"> <g> <path d="M24.4,24c-0.9,0-1.8-0.3-2.5-1l-5.8-5.7c-0.1-0.1-0.2-0.1-0.3,0L10.1,23c-1.4,1.4-3.6,1.4-5,0c-0.7-0.7-1-1.6-1-2.5 c0-1,0.4-1.8,1-2.5l9.4-9.3c0.9-0.9,2.3-0.9,3.1,0l9.4,9.3c0.7,0.7,1,1.6,1,2.5c0,1-0.4,1.8-1,2.5C26.2,23.7,25.3,24,24.4,24z"></path> </g> </g></svg></button>
      <button :class="{ 'clicked': isBackwardClicked }"><svg fill="#000000" height="100px" width="100px" version="1.1" id="Icons" xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" viewBox="0 0 32 32" xml:space="preserve" transform="matrix(1, 0, 0, 1, 0, 0)rotate(180)"><g id="SVGRepo_bgCarrier" stroke-width="0"></g><g id="SVGRepo_tracerCarrier" stroke-linecap="round" stroke-linejoin="round"></g><g id="SVGRepo_iconCarrier"> <g> <path d="M24.4,24c-0.9,0-1.8-0.3-2.5-1l-5.8-5.7c-0.1-0.1-0.2-0.1-0.3,0L10.1,23c-1.4,1.4-3.6,1.4-5,0c-0.7-0.7-1-1.6-1-2.5 c0-1,0.4-1.8,1-2.5l9.4-9.3c0.9-0.9,2.3-0.9,3.1,0l9.4,9.3c0.7,0.7,1,1.6,1,2.5c0,1-0.4,1.8-1,2.5C26.2,23.7,25.3,24,24.4,24z"></path> </g> </g></svg></button>
      <button :class="{ 'clicked': isRightClicked }"><svg fill="#000000" height="100px" width="100px" version="1.1" id="Icons" xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" viewBox="0 0 32 32" xml:space="preserve" transform="matrix(1, 0, 0, 1, 0, 0)rotate(90)"><g id="SVGRepo_bgCarrier" stroke-width="0"></g><g id="SVGRepo_tracerCarrier" stroke-linecap="round" stroke-linejoin="round"></g><g id="SVGRepo_iconCarrier"> <g> <path d="M24.4,24c-0.9,0-1.8-0.3-2.5-1l-5.8-5.7c-0.1-0.1-0.2-0.1-0.3,0L10.1,23c-1.4,1.4-3.6,1.4-5,0c-0.7-0.7-1-1.6-1-2.5 c0-1,0.4-1.8,1-2.5l9.4-9.3c0.9-0.9,2.3-0.9,3.1,0l9.4,9.3c0.7,0.7,1,1.6,1,2.5c0,1-0.4,1.8-1,2.5C26.2,23.7,25.3,24,24.4,24z"></path> </g> </g></svg></button>
    </div>
  </div>


</template>

<script>

export default {
name: 'Control',

components: {

},

data() {
  return {
    ws: null,
    messages: [],
    isForwardClicked: false,
    isBackwardClicked: false,
    isLeftClicked: false,
    isRightClicked: false
  }
},

// Se conectando ao websocket 
created() {
  this.ws = new WebSocket('ws://localhost:8000/ws');
  this.ws.onopen = () => {
    console.log('Connected to server');
  };
  this.ws.onmessage = (event) => {
    this.messages.push(event.data);
  };
},

// Os métodos enviam mensagens para o servidor
methods: {
  moveForward() {
    this.ws.send('move_forward');
    this.isForwardClicked = true;
    console.log('Moving Forward');
  },
  moveBackward() {
    this.ws.send('backward');
    this.isBackwardClicked = true;
    console.log('Moving Backward');
  },
  moveLeft() {
    this.ws.send('turn_left');
    this.isLeftClicked = true;
    console.log('Moving Left');
  },
  moveRight() {
    this.ws.send('turn_right');
    this.isRightClicked = true;
    console.log('Moving Right');
  },
  handleKeypress(event) {
    if (event.key === 'w' || event.key === 'ArrowUp' || event.key === 'W') {
      this.moveForward();
    } else if (event.key === 's' || event.key === 'ArrowDown' || event.key === 'S') {
      this.moveBackward();
    } else if (event.key === 'a' || event.key === 'ArrowLeft' || event.key === 'A') {
      this.moveLeft();
    } else if (event.key === 'd' || event.key === 'ArrowRight' || event.key === 'D') {
      this.moveRight();
    }
  },
  handleKeyup(event) {
    if (event.key === 'w' || event.key === 'ArrowUp' || event.key === 'W') {
      this.isForwardClicked = false;
      console.log('Stopped Moving Forward');
    } else if (event.key === 's' || event.key === 'ArrowDown' || event.key === 'S') {
      this.isBackwardClicked = false;
      console.log('Stopped Moving Backward');
    } else if (event.key === 'a' || event.key === 'ArrowLeft' || event.key === 'A') {
      this.isLeftClicked = false;
      console.log('Stopped Moving Left');
    } else if (event.key === 'd' || event.key === 'ArrowRight' || event.key === 'D') {
      this.isRightClicked = false;
      console.log('Stopped Moving Right');
    }
  }
},

beforeDestroy() {
  this.ws.close();
},

mounted() {
  window.addEventListener('keydown', this.handleKeypress);
  window.addEventListener('keyup', this.handleKeyup);
},

beforeDestroy() {
  window.removeEventListener('keydown', this.handleKeypress);
  window.removeEventListener('keyup', this.handleKeyup);
}
};
</script>

<style scoped>
  .top {
    background-color: #cce5ff;
    height: 50%;
    display: flex;
    align-items: center;
    justify-content: center;
    font-size: 20px;
  }


button {
background-color: transparent;
border: none;
cursor: pointer;
}

.clicked {
filter: invert(48%) sepia(79%) saturate(2476%) hue-rotate(86deg) brightness(118%) contrast(119%);
}
</style>