# AI Usage Log

Replace this template with your own entries. Add one entry per significant use of an AI tool.

## 1. <Understanding the codebase>

**What I asked:**
I asked it to help me understand how the code works.
**What I kept vs. rewrote, and why:**
N/A
**What the AI got wrong that I had to catch:**
N/A
**How I verified it ran correctly (not just that it compiled):**
N/A


## 2. <Troubleshooting why self.publish_odometry() was not working>

**What I asked:**
I asked what potential issues my code could be encountering with what I had.
**What I kept vs. rewrote, and why:**
I added "msg = Odometry()" and "self.odom_pub.publish(msg)." The former was added since I forgot to create the object before setting up the odometry message layout. The latter was added to publish the odometry.
**What the AI got wrong that I had to catch:**
N/A
**How I verified it ran correctly (not just that it compiled):**
The blue line appears on the simulation.


## 3. <Making a list of past GPS readings>

**What I asked:**
I asked it to make a list of past GPS readings and to use that history's baseline to make the heading more consistent.
**What I kept vs. rewrote, and why:**
Originally the AI got delta x and y using the oldest gps readings on the list, but I decided to find the average position of the list instead. I think this made the heading slightly more consistent.
**What the AI got wrong that I had to catch:**
I did not notice anything the AI did wrong.
**How I verified it ran correctly (not just that it compiled):**
I compared the AI's code to both my previous code and my edited code to see if there was a significant change in the amount of error of the odometry. I ran 2-3 tests with each of the different methods.


## 4. <next use>

**What I asked:**

**What I kept vs. rewrote, and why:**

**What the AI got wrong that I had to catch:**

**How I verified it ran correctly (not just that it compiled):**


## 5. <next use>

**What I asked:**

**What I kept vs. rewrote, and why:**

**What the AI got wrong that I had to catch:**

**How I verified it ran correctly (not just that it compiled):**


## 6. <next use>

**What I asked:**

**What I kept vs. rewrote, and why:**

**What the AI got wrong that I had to catch:**

**How I verified it ran correctly (not just that it compiled):**


## 7. <next use>

**What I asked:**

**What I kept vs. rewrote, and why:**

**What the AI got wrong that I had to catch:**

**How I verified it ran correctly (not just that it compiled):**