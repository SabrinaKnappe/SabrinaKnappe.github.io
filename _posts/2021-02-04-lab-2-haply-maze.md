---
layout: post
title: Lab 2- Intro to Haply
subtitle: Building a maze in processing for the haply.
cover-img: /assets/img/lab2/finishedmaze.png
thumbnail-img: /assets/img/lab2/finishedmaze.png
share-img: //assets/img/lab2/finishedmaze.png
tags: [haptics, coursework]
---
In this lab, we were tasked with setting up our Haply device, getting the software running, and modifying an existing example to make our own maze. I ran into a few issues during assembly, the biggest one being that the screwdriver included in the Haply kit was too small for the screws. I ended up buying a T6 screwdriver at home depot which did the trick (Thank you Linnea!)

![Assembly in progress](../assets/img/lab2/IMG_1855.jpg){:class="img-responsive"}
![Assembled Haply](../assets/img/lab2/IMG_1860.jpg){:class="img-responsive"}

I had originally set my gain to 10.0f since that was in the instructions, but it caused a bumpy sensation.
<iframe width="560" height="315" src="https://www.youtube.com/embed/i_07Q-8A3rs" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
I was able to fix this problem by setting the gain back to 1.0f at Antoine's recommendation.

Once I had all the software running, it was time for me to come up with an idea for the maze I wanted to build. I started by sketching what I thought a good maze might look like.
![maze sketch](../assets/img/lab2/IMG_0337.JPG){:class="img-responsive"}
Then I started experimenting with the pre-existing demos. At first I was working on the Hello Wall example until I found out we were allowed to use the fisica library, which simplifies everything a good deal.
![maze version 1](../assets/img/lab2/mazev1.png){:class="img-responsive"}
<iframe width="560" height="315" src="https://www.youtube.com/embed/2fles_LEUNg" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
While experimenting, I turned the block I was working on green so that I could tell which one I was moving. I removed all of the conditionals from the Maze Physics example since I just wanted a static maze. Once I had a version of the screen I liked without the extras from the example, I took the time to come up with a scale version of my maze to help me write it out.
![maze sketch 2](../assets/img/lab2/IMG_0338.JPG){:class="img-responsive"}

I also decided to make a star to mark the goal of the maze, which turned out to be more complicated than I initially thought. While there is a Star class in Processing, in order to make one with fisica you need to use the Polygon class and add vertices one by one to make your polygon. I used Desmos to map out points so that I could understand how to make a star in Processing. I was able to directly copy the coordinates I came up with in Desmos to use in Processing.
![maze version 1](../assets/img/lab2/Star.png){:class="img-responsive"}

<iframe width="560" height="315" src="https://www.youtube.com/embed/hxXN7hy0ifc" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

And here is the final product:
<iframe width="560" height="315" src="https://www.youtube.com/embed/rexgFfQcfU8" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>