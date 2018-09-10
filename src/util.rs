pub fn join_within_vec<I, E, F: Fn(&I, &I) -> Result<Option<I>, E>>(
    items: &mut Vec<I>,
    joiner: F,
) -> Result<(), E> {
    // do-until
    while {
        let mut could_join = false;

        let mut i = 0;

        while i + 1 < items.len() {
            let mut j = i + 1;

            while j < items.len() {
                if let Some(joined) =
                    joiner(&items[i], &items[j])?.or(joiner(&items[j], &items[i])?)
                {
                    items[i] = joined;
                    items.swap_remove(j);
                    could_join = true;
                } else {
                    j += 1;
                }
            }

            i += 1;
        }

        could_join
    } {}

    Ok(())
}