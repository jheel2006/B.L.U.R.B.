# [Blue Lovable Utilitarian Reading Buddy (B.L.U.R.B.)](https://intro.nyuadim.com/2024/12/10/final-project-blurb-the-book-buddy/)

[p5.js Sketch](https://editor.p5js.org/jheel2006/sketches/Ec3A_rwQI) &nbsp;|&nbsp; [Schematic Diagram](https://drive.google.com/file/d/1V71hiwQB9CMfAQ7V1h9kRwT-XXYfMTMg/view?usp=sharing)

<img width="1600" height="1200" alt="image" src="https://github.com/user-attachments/assets/cf190c80-de7f-45cd-acc0-276774a2335e" />


---

## Table of Contents
- [Inspiration](#inspiration)
- [Concept](#concept)
- [Implementation](#implementation)
  - [B.L.U.R.B.'s Design](#blurbs-design)
  - [Flipping Pages](#flipping-pages)
  - [Playing Ambient Music](#playing-ambient-music)
  - [Tracking Reading Time](#tracking-reading-time)
  - [Giving Break Suggestions](#giving-break-suggestions)
  - [Displaying Messages](#displaying-messages)
  - [Moving Head](#moving-head)
  - [Light Up Reading Lamp](#light-up-reading-lamp)
- [Overall Setup](#overall-setup)
- [User Testing](#user-testing)
- [Potential Improvements](#potential-improvements)
- [Reflections](#reflections)

---

## Inspiration

For all my assignments so far, I've tried to do something that's personal and meaningful to me in some way, and this project is no different. My favorite hobby, without a doubt, is reading. It's something I've loved for as long as I can remember. So when I started brainstorming for this project, I decided I wanted to create something that revolves around the joy of books and the experience of reading as a whole.

Of course, I've always thought there was something magical about getting lost in a story and exploring new worlds through words. But as much as I love reading, it can sometimes feel like a solitary experience, especially when I come across a thought or idea I want to share but no one's around to talk to. That's what inspired me to create a book buddy robot: something that could bring a sense of companionship to reading, making it feel a little less lonely while still keeping the focus on the stories I love so much.

## Concept

The concept behind my book buddy robot, B.L.U.R.B., is to create a small, interactive companion that enhances your reading experience. Made using **p5.js** and **Arduino**, B.L.U.R.B. is designed to be there with you as you dive into a good book, offering a few fun features along the way.

| Feature | Platform | Description |
|---|---|---|
| **Flip Pages** | Arduino | Automatically flips the pages of your book, keeping the flow going without having to pause and turn the page yourself. It feels like having a personal assistant handle the mundane task so you can stay immersed in your story. |
| **Play Ambient Music** | p5.js | Plays soft, ambient music to set the atmosphere, making it easier to focus and get lost in your book. You can choose different moods depending on the type of book you're reading. |
| **Track Reading Time** | p5.js | Displays a timer on screen to track how long you've been reading, helping you set reading goals or simply monitor your sessions without taking away from the reading experience. |
| **Give Break Suggestions** | p5.js | When you take a break, B.L.U.R.B. suggests refreshing activities like stretching, taking a short walk, drinking water, or doodling. |
| **Display Messages** | Arduino | An LCD screen displays small, personalized messages: B.L.U.R.B. saying "Hello", and whether you're currently reading or on break. |
| **Move Head** | Arduino | Whenever you switch between reading and break modes, B.L.U.R.B. moves his head from side to side, making him feel more like a real companion. |
| **Light Up Reading Lamp** | Arduino | A small reading lamp that turns on or off with a tap on B.L.U.R.B.'s foot, for when the lights are dim or you need a little extra light. |

---

## Implementation

### B.L.U.R.B.'s Design

B.L.U.R.B.'s design started with a cute 3D model found on Thingiverse. However, 3D printing issues meant the model had to be split into multiple parts, which were glued together later. After printing and assembling the pieces, I spray-painted all the components a soft pastel blue to match B.L.U.R.B.'s calming design.

This is a video I took after the initial 3D printing stages, while working on making his head move from side to side:

<!-- ADD VIDEO HERE: head movement test after 3D printing (0:24) -->

On the p5 side, B.L.U.R.B. sits on a cozy sofa in a softly lit library room. The setting is designed to create a comfortable atmosphere that enhances the reading experience. B.L.U.R.B. talks to the user through animated speech bubbles, with the text appearing in a typewriter effect. This combination of design and animation gives B.L.U.R.B. a friendly, approachable personality while keeping the focus on a relaxing, cozy environment.

<img width="1200" height="726" alt="image" src="https://github.com/user-attachments/assets/ea751112-15e0-444a-8450-be2525345964" />


### Flipping Pages

To figure out how to flip pages mechanically, I watched a few videos online for inspiration. My initial idea was to use two servo motors and a temporary adhesive, like Blu Tack, that doesn't dry out too easily. I built several prototypes and tested different adhesives, but none worked perfectly. Strong adhesives stuck too firmly to the paper and couldn't unstick, while weaker ones didn't have enough grip to flip a page.

The adhesive would be stuck on the lower end of the shorter stick of the prototype below:

<!-- ADD VIDEO HERE: adhesive prototype (0:54) -->

Eventually, I tried a different approach: a combination of a wheel and a stick attached to a servo motor.

1. The wheel makes contact with the page and rotates clockwise for some time to lift it.
2. The stick slides into the gap created by the lifted page and swings 180° to complete the flip, then returns to its original position.
3. The wheel briefly rotates counterclockwise to smooth the next page down so it lies flat.

The wheels available in the lab didn't work well for this setup, so I had a custom wheel 3D printed. This improved the mechanism somewhat, but it still wasn't perfect. The main issue was that the wheel would sometimes lift multiple pages at once, and the system didn't have the power to flip them all.

It took a lot of iterations, but the mechanism worked much better in the end with one small change: in the first few seconds of the wheel running (when it has likely lifted the first page slightly), the stick moves slightly inward to fill this gap. The wheel then runs a little more, and the servo motion happens as before. This makes sure the stick (more likely than not) ends up in the right spot, under just one page, so the servo flips only the page above it.

<!-- ADD VIDEO HERE: final page-flipping mechanism (0:33) -->

### Playing Ambient Music

For this feature, built into the p5 part of the project, I added three sound files: **Lofi**, **Nature**, and **Piano**. The user can choose between them whenever they want ambient music while reading, switch between sounds, or stop the music entirely, giving them full control over their listening experience. These sounds were chosen because they're calming and help create a focused atmosphere without being distracting.

### Tracking Reading Time

To help users monitor how long they spend reading, I added a reading timer. The timer starts automatically when the user clicks the **"Begin Reading"** button in p5. If the user takes a break, the timer pauses and saves the current reading time. Once the user clicks to continue reading, the timer resumes from where it left off.

This feature works in the background, so the user can focus on their reading without keeping track of time manually. It's a simple way to measure reading sessions, making it easy to reflect on reading habits or set goals.

### Giving Break Suggestions

Whenever the user takes a break, B.L.U.R.B. adds a personal touch by saying:

> "Enjoy your break! I suggest you [insert break suggestion] :)"

All of these messages show up in p5. The break suggestion is randomly selected from a list of about 40 healthy activities stored in a CSV file, curated to encourage wellness and productivity during breaks. The suggestions include ideas like:

- Doing a quick exercise routine
- Grabbing a healthy snack
- Tidying up your desk
- Meditating for a few minutes
- Sketching or doodling
- Taking a short walk

This feature adds personality to B.L.U.R.B. and also helps users make the most of their breaks by promoting healthy habits and keeping their minds refreshed.

### Displaying Messages

On the p5 side, B.L.U.R.B. introduces himself to the user and briefly explains the features he offers and how he can help with reading.

On the Arduino side, an LCD screen attached to B.L.U.R.B.'s neck displays messages and adds personality:

- **On startup:** a cheerful "Hellooo!" greets the user.
- **When reading begins or resumes:** the screen shows "Reading Time" with a beating heart animation. The heart adds a sense of life to the robot and beats in time with the seconds on the reading timer.
- **During a break:** the screen switches to a friendly "Break Time :)" message, creating a fun and relaxing vibe during the pause.

### Moving Head

To make the experience more interactive, B.L.U.R.B. moves his head from side to side whenever the user switches between reading and break modes. This movement is controlled by a servo motor attached to his head, adding a playful and lifelike element to the robot.

### Light Up Reading Lamp

A blue NeoPixel ring acts as a reading lamp, positioned near the book to provide enough light. Its color matches B.L.U.R.B.'s blue design for a cohesive look. The lamp is controlled by a piezo pressure sensor on B.L.U.R.B.'s foot, which is marked with a lamp sticker. Tapping the foot toggles the light on or off, so the user can control the lamp without interrupting their reading. The soft blue light adds a calming and functional touch to the environment.

<img width="1200" height="900" alt="image" src="https://github.com/user-attachments/assets/b2a14b8e-dc83-4b85-8a8a-3060633ebbd3" />

Initial prototype:

<!-- ADD VIDEO HERE -->

### Work in Progress Pictures

<img width="1024" height="461" alt="image" src="https://github.com/user-attachments/assets/a36c9bc3-8107-468e-9a9a-c94794a60270" />

<img width="1024" height="461" alt="image" src="https://github.com/user-attachments/assets/739e7a5d-0b3c-40d0-99d6-662fc7daeea1" />

<img width="1024" height="461" alt="image" src="https://github.com/user-attachments/assets/d0ba97c4-ef74-4b65-a2c8-6f1c7a139f05" />


---

## Overall Setup

The overall setup was designed with both functionality and looks in mind. A wooden plank of the right size was covered with brown paper, creating a smooth surface to lay the book on and house the flipping mechanism. The mechanism includes a wheel with a motor attached to a block of wood fixed to the plank, along with a servo mounted on the bottom edge to control the page flipping.

To keep the wiring organized, I used a separate box, also covered in brown paper, to house all the electronics. I cut slits in the box so the wiring could extend outside where needed. To add a touch of personality, I printed cute book-related quotes that look like stickers, cut them out, and stuck them on the box. B.L.U.R.B. sits on top of the box, completing the setup. This design keeps everything neatly organized while also adding to the overall look of the reading space.

<img width="1200" height="675" alt="image" src="https://github.com/user-attachments/assets/9081798c-0251-4e14-8c1d-9ed6940f5638" />


---

## User Testing

This project was displayed at the **Fall 2024 end-of-semester Interactive Media Showcase at NYU Abu Dhabi**. Here are some photos and videos of people interacting with it at the showcase:

<!-- ADD VIDEO HERE: showcase interaction (1:15) -->
<img width="1200" height="900" alt="image" src="https://github.com/user-attachments/assets/829a58bd-fdd6-478c-b1b8-e56370005272" />


---

## Potential Improvements

There are a few areas where I could improve the project to make it even more engaging:

- **More interaction:** have B.L.U.R.B. move his head randomly to keep the user engaged.
- **Idle mode:** if the p5 screen is left idle for a while, hide everything except the timer to create a more minimal, focused reading environment.
- **Better lighting:** position the reading lamp above the book for better light coverage across the pages.

These changes would improve both the functionality and the user experience, making the project even more dynamic and enjoyable :))

---

## Reflections

I'm really proud and happy with how this project turned out! When I first decided on the idea, I had no clue whether it would be possible to bring everything together and make it all work. But I was determined to give it my best and worked on it every day, tackling one challenge at a time. The most difficult part, of course, was the page-flipping mechanism. I spent several days experimenting with different approaches before I finally got it to work.

In the end, seeing all the different components come together felt incredibly rewarding. Presenting it at the showcase was a really proud moment for me. Watching people interact with B.L.U.R.B. and enjoy the experience I had created made all the effort worth it. I'm happy with how the project reflects my passion for reading and combines functionality with a cute, thoughtful design. It's been a journey full of challenges, but the result really does feel like an accomplishment :))
