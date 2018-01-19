import ca._4976.motion.data.Moment;
import ca._4976.motion.data.Profile;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

import static ca._4976.motion.data.Profile.Format.*;

public class Test {

    public static void main(String... args) {

        DateFormat dateFormat = new SimpleDateFormat("YYYY.MM.DD");

        Profile profile = new Profile(
                "Test",
                dateFormat.format(new Date()),
                new Moment[] {
                        new Moment(new Integer[0], new Double[] { .0, .0 }, new Double[] { 1.0, 1.0 }, new Double[] { 1.0, -.1 }),
                        new Moment(new Integer[0], new Double[] { 1.0, .0 }, new Double[] { 1.0, .0 }, new Double[] { 1.0, -.1 }),
                        new Moment(new Integer[0], new Double[] { 1.0, 1.0 }, new Double[] { .0, .0 }, new Double[] { 1.0, -.1 }),
                        new Moment(new Integer[0], new Double[] { 1.0, 1.0 }, new Double[] { .0, .0 }, new Double[] { 1.0, -.1 }),
                        new Moment(new Integer[0], new Double[] { 1.0, .0 }, new Double[] { 1.0, .0 }, new Double[] { 1.0, -.1 }),
                        new Moment(new Integer[0], new Double[] { .0, .0 }, new Double[] { 1.0, 1.0 }, new Double[] { 1.0, -.1 })
                }
        );

        String serialized = profile.serialize(CSV);
        String deserialied = Profile.deserialize(serialized, CSV).serialize(CSV);

        System.out.println(serialized + "\n");

        System.out.println(Profile.deserialize(serialized, CSV).serialize(CSV) + "\n");

        System.out.println(deserialied.equals(serialized) ? "Worked" : "Didn't Work");
    }
}
