import javax.sound.sampled.*;

public class BeepHelper {
  public static void playBeep(float freqHz, int durMs, float vol) {
    try {
      int sampleRate = 44100;
      int nSamples   = (int)(sampleRate * durMs / 1000.0);
      byte[] buf = new byte[nSamples * 2];
      for (int i = 0; i < nSamples; i++) {
        int attSamples = (int)(sampleRate * 0.010);
        int relSamples = (int)(sampleRate * 0.010);
        float env = 1.0f;
        if (i < attSamples) env = (float)i / attSamples;
        else if (i > nSamples - relSamples) env = (float)(nSamples - i) / relSamples;
        double angle = 2.0 * Math.PI * freqHz * i / sampleRate;
        short sample = (short)(Math.sin(angle) * 32767 * vol * env);
        buf[2*i]   = (byte)(sample & 0xFF);
        buf[2*i+1] = (byte)((sample >> 8) & 0xFF);
      }
      AudioFormat fmt = new AudioFormat(sampleRate, 16, 1, true, false);
      SourceDataLine sdl = AudioSystem.getSourceDataLine(fmt);
      sdl.open(fmt, buf.length);
      sdl.start();
      sdl.write(buf, 0, buf.length);
      sdl.drain();
      sdl.close();
    } catch (Exception e) {
      System.out.println("Error de audio: " + e.getMessage());
    }
  }

  public static void playSequence(float freq1, int dur1, float freq2, int dur2, float vol) {
    playBeep(freq1, dur1, vol);
    try { Thread.sleep(60); } catch (Exception e) {}
    playBeep(freq2, dur2, vol);
  }
}
